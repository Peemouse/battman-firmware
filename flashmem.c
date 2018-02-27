#include "flashmem.h"
#include "ch.h"
#include "hal.h"
#include <string.h>

#define PEC_POLY 7


//COPY/PASTE from LTC6803.c

/*
 *  * SPI configuration (562kHz, CPHA=1, CPOL=1, MSb first).
 *   */
static const SPIConfig ls_spicfg = {
    NULL,
    GPIOA,
    2,
    SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPOL | SPI_CR1_CPHA,
    SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};

static uint32_t workingHours;
static uint32_t totalAh;

static void flashmem_readdata(uint8_t config[6]);
static void flashmem_writedata(uint8_t *data, uint8_t page);

static void flasmem_wren(); //Command :  enable Write bit
static void flasmem_wrdi(); //Command :  disable Write bit
static uint32_t flashmem_read(uint32_t address); //Command :  read
static void flashmem_pp(uint32_t data, uint32_t address); //Command :  page program

static void spi_sw_transfer(char *in_buf, const char *out_buf, int length);

bool flashmem_wip; //Write in progress : don't send other commands
bool flashmem_wel; //write enable latch : ready to write. To enable before writing, disable after writing.

void flashmem_init(void)
{
}

void flashmem_update(void)
{
}

uint32_t flashmem_get_workinghours() {
	workingHours = flashmem_readdata(TOTAL_ACTIVEHOURS);
	return workingHours;
}

uint32_t flashmem_get_totalAmpHours() {
	totalAh = flashmem_readdata(TOTAL_AH_CONS);
	return totalAh;
}

void flashmem_write_workinghours(uint32_t value) {
	flashmem_writedata(TOTAL_ACTIVEHOURS, value);
	return;
}

void flashmem_write_totalAmpHours(uint32_t value) {
	flashmem_writedata(TOTAL_AH_CONS, value);
	return;
}

uint32_t flashmem_readdata(uint32_t add) {
	uint8_t tryCounter=0;
	uint32_t value=0;
	
	flashmem_rdsr();
	if (flashmem_wip) {
		return;
	}

	value = flashmem_read(add);
	return value;
}

uint32_t flashmem_writedata(uint32_t add, uint32_t data) {
	uint8_t tryCounter=0;
	uint32_t value=0;
	
	flashmem_rdsr();
	if (flashmem_wip) {
		return;
	}
	while (!flashmem_wel && tryCounter < 3) {
		flashmem_wren();
		flashmem_rdsr();
		tryCounter++;
	}
	if (flashmem_wel) {
		flashmem_pp(add, data);
		flashmem_wrdi();
		return;
	}
	else {
		return;
	}
}

static void flashmem_rdsr() { //Command :  read status register
	 uint8_t txbuf[1];
	 uint8_t rxbuf[1];
	 txbuf[0] = 0x05;
	 
	spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 1);
#else
    spiExchange(&SPID1, 1, txbuf, rxbuf);
#endif
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */
	
	flashmem_wip=rxbuf[1] & 0x01;
	flashmem_wel=rxbuf[1] & 0x02;
}

static void flashmem_wren() //Command :  enable Write bit
{
    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    txbuf[0] = 0x06;
    
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 1);
#else
    spiExchange(&SPID1, 1, txbuf, rxbuf);
#endif
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */
}

static void flashmem_wrdi() //Command :  disable Write bit
{
    uint8_t txbuf[1];
    uint8_t rxbuf[1];
    txbuf[0] = 0x04;
    
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 1);
#else
    spiExchange(&SPID1, 1, txbuf, rxbuf);
#endif
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */
}

static uint32_t flashmem_read(uint32_t address) //Command :  disable Write bit
{
    uint8_t txbuf[4];
    uint8_t rxbuf[4];
	uint32_t data;
	
    txbuf[0] = 0x04;
	txbuf[1] = address >> 8;
	txbuf[2] = address >> 4 & 0x00F;
	txbuf[3] = address & 0x000F;
    
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 4);
#else
    spiExchange(&SPID1, 4, txbuf, rxbuf);
#endif

	txbuf[0]=0xFF;
	rxbuf[];{0,0,0,0};
	txbuf
	for (int i=3; i>=0;i--) {
		spiExchange(&SPID1, 1, txbuf, rxbuf[0]);
		data = data + rxbuf[0] << (8 * i);
	}
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */

	return data;
}

static void flashmem_pp(uint32_t data, uint32_t address) //Command :  disable Write bit
{
    uint8_t txbuf[4];
    uint8_t rxbuf[4];
	
    txbuf[0] = 0x02;
	txbuf[1] = address >> 8;
	txbuf[2] = address >> 4 & 0x00F;
	txbuf[3] = address & 0x000F;
    
    spiAcquireBus(&SPID1);
    spiStart(&SPID1, &ls_spicfg);
    spiSelect(&SPID1);
#ifdef BATTMAN_4_0
    spi_sw_transfer(rxbuf, txbuf, 4);
#else
    spiExchange(&SPID1, 4, txbuf, rxbuf);
#endif

	txbuf[0] = address >> 12;
	txbuf[1] = address >> 8 & 0x0F;
	txbuf[2] = address >> 4 & 0x00F;
	txbuf[3] = address & 0x000F;
	
	for (int i=0; i<4; i++) {
		spiExchange(&SPID1, 1, txbuf, rxbuf);
	}
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);              /* Ownership release.               */
	
	//TODO : read the written value to confirm the success of writing

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

