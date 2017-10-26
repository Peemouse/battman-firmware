#include "ltc6803.h"
#include "ch.h"
#include "hal.h"
#include <string.h>
#include "config.h"
#include "faults.h"

#define PEC_POLY 7

/*
 *  * SPI configuration (562kHz, CPHA=1, CPOL=1, MSb first).
 *   */
static const SPIConfig ls_spicfg = {
    NULL,
    GPIOA,
    4,
    SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
    SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

static volatile Config *config;
static float cells[12];
static float ltc6803Temp[3];
static uint8_t configReg[6];
static bool lock = false;
static bool ltc6803THSD = false;
static bool muxFail = false;
static float refVoltage;

static volatile systime_t conversionStart;
static void ltc6803_wrcfg(uint8_t config[6]);
static void ltc6803_stcvad(void);
static void ltc6803_rdcv(float cells[12]);
static void ltc6803_rdtmp(float ltc6803Temp[3]); //bool ltc6803THSD
static void ltc6803_sttmpad(void);
static void ltc6803_dagn(void);
static void ltc6803_rddgnr(void);
static bool ltc6803_checkVoltage(float diagVoltage);
static uint8_t pec8_calc(uint8_t len, uint8_t *data);
static void spi_sw_transfer(char *in_buf, const char *out_buf, int length);

void ltc6803_init(void)
{
    config = config_get_configuration();
#ifdef BATTMAN_4_0
    palSetPadMode(SCK_GPIO, SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(MISO_GPIO, MISO_PIN, PAL_MODE_INPUT_PULLUP | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(MOSI_GPIO, MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
#endif
    configReg[0] = 0b01100001;
    configReg[1] = 0b00000000;
    configReg[2] = 0b00000000;
	if (config->numCells > 4) {
		configReg[3] = 0b11111111 << (config->numCells-4); //LTC6803 : Masks cells according to the battery config
	}
	else {
		configReg[3] = 0b11111111;	
	}
    configReg[4] = 0b00000000;
    configReg[5] = 0b00000000;
	
	ltc6803_wrcfg(configReg);
	
    conversionStart = chVTGetSystemTime();
	
	ltc6803_diagnostic();
}

void ltc6803_update(void)
{
    //ltc6803_wrcfg(configReg);
    if (ST2MS(chVTTimeElapsedSinceX(conversionStart)) > 13)
    {
		//TODO : implement OPEN WIRE detection (page 28 of LTC6803 datasheet) (STOWAD ->RDCV:store in CELLA(n) -> STOWAD -> RDCV:store in CELLB(n). CELLA(n+1)-CELLB(n+1) > 200mV or CELLLB(n+1) = 5.375V : cell n is open wire)
		//Also a pack voltage comparison with sum of all cells should be performed.
        ltc6803_rdcv(cells);
        ltc6803_stcvad();
        chThdSleepMicroseconds(13);//Time needed by the LTC6803 to execute measurements
		ltc6803_rdtmp(ltc6803Temp);
		ltc6803_sttmpad();
        conversionStart = chVTGetSystemTime();
    }
}

float* ltc6803_get_cell_voltages(void)
{
	
    return cells;
}

float* ltc6803_get_temp(void) {
	
    return ltc6803Temp;
}

void ltc6803_enable_balance(uint8_t cell)
{
    if (lock)
        return;
    if (cell > config->numCells)
        return;
    if (cell <= 8)
    {
        configReg[1] |= 1 << (cell - 1);
    }
    else
    {
        configReg[2] |= 1 << (cell - 9);
    }
	ltc6803_wrcfg(configReg);
}

void ltc6803_disable_balance(uint8_t cell)
{
    if (lock)
        return;
    if (cell > config->numCells)
        return;
    if (cell <= 8)
    {
        configReg[1] &= ~(1 << (cell - 1));
    }
    else
    {
        configReg[2] &= ~(1 << (cell - 9));
    }
	ltc6803_wrcfg(configReg);
}

void ltc6803_disable_balance_all(void)
{
    if (lock){
        return;
	}
    configReg[1] = 0;
    configReg[2] = 0;
	ltc6803_wrcfg(configReg);
}

void ltc6803_diagnostic(void) {
	uint32_t result=0; //First 12 bits for cells, 4 bits for 2x external temp, internal temp and THSD bit, 1 bit refVoltage, 1 bit for muxFail
	uint8_t bitCounter=0;
	
	ltc6803_dagn(); //Execute diagnostic command
	chThdSleepMicroseconds(20); //Wait for its execution
	ltc6803_rdcv(cells); //Read cells voltage registers
	ltc6803_rdtmp(ltc6803Temp); //Read temperature voltage registers
	ltc6803_rddgnr(); //read Diagnostic registers
	
	for (uint8_t i=0; i < config->numCells; i++) {
		result = (ltc6803_checkVoltage(cells[i]) << bitCounter);
		bitCounter++;
	}
	for (uint8_t i=0; i < 3; i++) {
		result = (ltc6803_checkVoltage(ltc6803Temp[i]) << bitCounter);
		bitCounter++;
	}
	result = ltc6803THSD << bitCounter;
	bitCounter++;
	result = (ltc6803_checkVoltage(refVoltage) << bitCounter);
	bitCounter++;
	result = muxFail << bitCounter;

	if (result != 0) {
		faults_set_warning(WARNING_LTC6803_ERROR);
	}
	
	ltc6803_stcvad(); //Refresh registers with operationnal values
	ltc6803_sttmpad();
}

void ltc6803_lock(void)
{
    lock = true;
}

void ltc6803_unlock(void)
{
    lock = false;
}

static void ltc6803_wrcfg(uint8_t config[6]) //LTC6803 command : Write config
{
    const uint8_t BYTES_IN_REG = 6;
    const uint8_t CMD_LEN = 9;
    uint8_t cmd[9];
    uint8_t rxbuf[9];
    uint16_t cfg_pec;
    uint8_t cmd_index;

    cmd[0] = 0x01;
    cmd[1] = 0xC7;

    cmd_index = 2;
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
        cmd[cmd_index] = config[current_byte];
        cmd_index++;
    }

    cfg_pec = pec8_calc(BYTES_IN_REG, &config[0]);    // calculating the PEC for each ICs configuration register data
    cmd[cmd_index] = (uint8_t)cfg_pec;
    cmd_index++;

    spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
    spiStart(&SPID1, &ls_spicfg);       /* Setup transfer parameters.       */
    spiSelect(&SPID1);                  /* Slave Select assertion.          */
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, cmd, CMD_LEN);
#else
    spiExchange(&SPID1, CMD_LEN, cmd, rxbuf);
#endif

    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */
}

static void ltc6803_stcvad(void) //LTC6803 command: Start Cell Voltage ADC Conversions and Poll Status
{
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    txbuf[0] = 0x10;
    txbuf[1] = 0xB0;
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 2);
#else
    spiExchange(&SPID1, 2, txbuf, rxbuf);
#endif
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */
}

static void ltc6803_rdcv(float cells[12]) //LTC6803 command: Read cells voltage from registers
{
    int data_counter = 0;
    int pec_error = 0;
    uint8_t data_pec = 0;
    uint8_t received_pec = 0;
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    uint8_t rx_data[19];
    txbuf[0] = 0x04;
    txbuf[1] = 0xDC;

    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 2);
#else
    spiExchange(&SPID1, 2, txbuf, rxbuf);
#endif
    txbuf[0] = 0xFF;
	
    for (int j = 0; j < 19; j++){
#ifdef BATTMAN_4_0
        spi_sw_transfer(rxbuf, txbuf, 1);
#else
        spiExchange(&SPID1, 1, txbuf, rxbuf);
#endif
	
		rx_data[j] = rxbuf[0];
    }
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */

    data_counter = 0;
    uint16_t byteLow, byteHigh;

    received_pec = rx_data[18];
    data_pec = pec8_calc(18, &rx_data[0]);
    if (received_pec != data_pec){
        pec_error = -1;
		faults_set_warning(WARNING_LTC6803_ERROR);
		return;
    }

    for (int k = 0; k < 12; k = k + 2){
        byteLow = rx_data[data_counter++];

        byteHigh = (uint16_t)(rx_data[data_counter] & 0x0F) << 8;

        cells[k] = (float)(byteLow + byteHigh - 512) * 1.5 / 1000.0;
        byteHigh = (rx_data[data_counter++]) >> 4;

        byteLow =  (rx_data[data_counter++]) << 4;

        cells[k + 1] = (float)(byteLow + byteHigh - 512) * 1.5 / 1000.0;
    }
    //check values to detect open wire. If so, value = -1V.
    for (int l = 0; l < config->numCells; l++) {
    	if (cells[l]>5.374) {
    		cells[l]=-1.0;
    	}
    }
}
static void ltc6803_sttmpad(void) { //LTC6803 command: Start temperature ADC Conversions and Poll Status
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    txbuf[0] = 0x30;
    txbuf[1] = 0x50;
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 2);
#else
    spiExchange(&SPID1, 2, txbuf, rxbuf);
#endif
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */	
}


static void ltc6803_rdtmp(float ltc6803Temp[3]) { //LTC6803 command: Read temperatures from registers

int data_counter = 0;
    int pec_error = 0;
    uint8_t data_pec = 0;
    uint8_t received_pec = 0;
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    uint8_t rx_data[6];
    txbuf[0] = 0x0E;
    txbuf[1] = 0xEA;

    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 2);
#else
    spiExchange(&SPID1, 2, txbuf, rxbuf);
#endif
    txbuf[0] = 0xFF;
    for (int j = 0; j < 6; j++)
    {
#ifdef BATTMAN_4_0
        spi_sw_transfer(rxbuf, txbuf, 1);
#else
        spiExchange(&SPID1, 1, txbuf, rxbuf);
#endif
        rx_data[data_counter++] = rxbuf[0];
    }
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */

    data_counter = 0;
    uint16_t byteLow, byteHigh;

    received_pec = rx_data[6];
    data_pec = pec8_calc(6, &rx_data[0]);
	
    if (received_pec != data_pec)
    {
        pec_error = -1;
		return;
    }
	
    for (int k = 0; k < 3; k = k + 2)
    {
        byteLow = rx_data[data_counter++];
        byteHigh = (uint16_t)(rx_data[data_counter] & 0x0F) << 8;
        ltc6803Temp[k] = (float)(byteLow + byteHigh - 512) * 1.5;
        
        byteHigh = (rx_data[data_counter++]) >> 4;
        if (k==2){
        	if (rx_data[data_counter] & (1 << 4)) {ltc6803THSD=true;}//Check the hardware temperature fault of LTC6803 (145 deg)
			else {ltc6803THSD=false;}			
        }
        byteLow =  (rx_data[data_counter++]) << 4;
        ltc6803Temp[k + 1] = (float)(byteLow + byteHigh - 512) * 1.5;
    }
 }

static void ltc6803_dagn(void) { //LTC6803 command: Start Diagnose and Poll Status
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    txbuf[0] = 0x52;
    txbuf[1] = 0x74;
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 2);
#else
    spiExchange(&SPID1, 2, txbuf, rxbuf);
#endif
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */	
}

static void ltc6803_rddgnr(void) { //LTC6803 command: Read reference voltage

int data_counter = 0;
    int pec_error = 0;
    uint8_t data_pec = 0;
    uint8_t received_pec = 0;
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    uint8_t rx_data[3];
    txbuf[0] = 0x54;
    txbuf[1] = 0x6B;

    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 2);
#else
    spiExchange(&SPID1, 2, txbuf, rxbuf);
#endif
    txbuf[0] = 0xFF;
    for (int j = 0; j < 3; j++)
    {
#ifdef BATTMAN_4_0
        spi_sw_transfer(rxbuf, txbuf, 1);
#else
        spiExchange(&SPID1, 1, txbuf, rxbuf);
#endif
        rx_data[data_counter++] = rxbuf[0];
    }
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */

    data_counter = 0;
    uint16_t byteLow, byteHigh;

    received_pec = rx_data[3];
    data_pec = pec8_calc(2, &rx_data[0]);
	
    if (received_pec != data_pec)
    {
        pec_error = -1;
		faults_set_warning(WARNING_LTC6803_ERROR);
    }
	
	byteLow = rx_data[data_counter++];
	byteHigh = (uint16_t)(rx_data[data_counter] & 0x0F) << 8;
	refVoltage = (float)(byteLow + byteHigh - 512) * 1.5;

	if (rx_data[data_counter] & (1 << 5)) {muxFail=true;} //Check the MUX fail bit
	else {muxFail=false;}		
}

static bool ltc6803_checkVoltage(float diagVoltage) { //Check if the voltage is in the range 2.5V +-16%
	float upperLimit,lowerLimit;
	bool isGood=true;
	
	upperLimit = 2.5*1.16;
	lowerLimit = 2.5*0.84;
	if (diagVoltage<lowerLimit || diagVoltage>upperLimit) {
		isGood=false;
	}
	return isGood;
}
 
static void spi_sw_transfer(char *in_buf, const char *out_buf, int length) {
    palSetPad(SCK_GPIO, SCK_PIN);
    chThdSleepMicroseconds(100);

    for (int i = 0; i < length; i++) {
        unsigned char send = out_buf ? out_buf[i] : 0;
        unsigned char receive = 0;

        for (int bit=0; bit < 8; bit++) {
            palClearPad(SCK_GPIO, SCK_PIN);
            palWritePad(MOSI_GPIO, MOSI_PIN, send >> 7);
            send <<= 1;

            chThdSleepMicroseconds(100);

            receive <<= 1;
            if (palReadPad(MISO_GPIO, MISO_PIN)) {
                receive |= 0x1;
            }

            palSetPad(SCK_GPIO, SCK_PIN);
            chThdSleepMicroseconds(100);
        }

        if (in_buf) {
            in_buf[i] = receive;
        }
    }
}

static uint8_t pec8_calc(uint8_t len, uint8_t *data)
{
    uint8_t  remainder = 0x41;//PEC_SEED;
    int byte;
    uint8_t bit;
    /*
     * Perform modulo-2 division, a byte at a time.
     */
    for (byte = 0; byte < len; ++byte)
    {
        /*
         * Bring the next byte into the remainder.
         */
        remainder ^= data[byte];

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (bit = 8; bit > 0; --bit)
        {
            /*
             * Try to divide the current data bit.
             */
            if (remainder & 128)
            {
                remainder = (remainder << 1) ^ PEC_POLY;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
    }

    /*
     * The final remainder is the CRC result.
     */
    return (remainder);

}

