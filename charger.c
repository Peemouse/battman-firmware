#include "charger.h"
#include "hal.h"
#include "hw_conf.h"
#include "config.h"
#include "current_monitor.h"
#include "power.h"
#include "ltc6803.h"
#include "analog.h"
#include <math.h>
#include "faults.h"

#define I2C_ADDRESS 0x1F

static volatile Config *config;
static volatile float charge_voltage;
static volatile bool isCharging = false;
static volatile float input_voltage;
static volatile float output_voltage;
static volatile bool charge_enabled = false;
static volatile bool storage = false;
static volatile float current_control_integral = 0.0;
static volatile systime_t lastTime;
static volatile bool balancing = false;
static volatile systime_t balanceUpdateTime;
static volatile bool startCharging = false;
static volatile uint8_t stepCCStartingCounter = 0;
static volatile bool chargeComplete = false;
static volatile uint8_t chargeCompleteCounter = 0;
static volatile float chargeCurrentSetpoint = 0.0;
static volatile float rampUpCurrent = 0.5 //when starting a charge, ramp up from 0A to config->chargeCurrent. Unit : amp per second
static volatile bool isBalancingCell[12] = {false};
static volatile float mahCount = 0.0;
static volatile systime_t mahCountDeltaTime;

static void set_voltage(float voltage);

void charger_init(void)
{
    config = config_get_configuration();
    //palSetPad(CHG_SW_GPIO, CHG_SW_PIN); //TBC
	palClearPad(CHG_SW_GPIO, CHG_SW_PIN); //TBC
#ifdef BATTMAN_4_0
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = 0x07 << 2;
    tx[1] = 0x01 << 1; //0x01 : address of i2c slave device (DAC)
    i2cMasterTransmitTimeout(&I2C_DEV, 0x2E, tx, 2, rx, 0, MS2ST(10));
#endif
    set_voltage(current_monitor_get_bus_voltage() + 0.2); //Initializing charging voltage to battery voltage + hysteresis to avoid peak of current
    lastTime = chVTGetSystemTime();
    balanceUpdateTime = chVTGetSystemTime();
}

void charger_update(void)
{
    input_voltage = analog_charger_input_voltage();
	float endOfCharge_voltage = 0.0;
	
    //TODO : enable charging only if battery voltage is a certain amount below the charge voltage (ex : chargevoltage = 4.20V/cell, trigger a charge only if 4.00V/cell)
	
	//Faults checking + charge enabled
	if (!charge_enabled || chargeComplete || faults_check_fault(FAULT_CELL_OV) || faults_check_fault(FAULT_BATTERY_OV) || faults_check_fault(FAULT_OVERCURRENT) || faults_check_fault(FAULT_BATTERY_TEMP) || faults_check_fault(FAULT_BOARD_TEMP) ) {
		palClearPad(CHG_SW_GPIO, CHG_SW_PIN);
		startCharging = false;
		isCharging = false;
		current_control_integral = 0.0;
		balancing = false;
		
	}
    else if (charge_enabled && !chargeComplete && (startCharging || isCharging)) {
        powerSwitchOff();
		
        float currentErr;
        float chargeVoltage;
        float dt = ST2US(chVTTimeElapsedSinceX(lastTime)) / 1e6;
        float* cells;
        cells = ltc6803_get_cell_voltages();
        float highestCellV = 0.0;
        float lowestCellV = 999.0;
        uint8_t lowestCellNum;
		
		if (storage) {
			endOfCharge_voltage = config->cellStorageVoltage * config->numCells;
		}
		else {
			endOfCharge_voltage = config->cellEndChargeVoltage * config->numCells;
		}
			
		switch(config->chargeMode) {
			
			case CURRENT_CONTROL:
				if (startCharging) { //Ramp up
					if (stepCCStartingCounter == 0) {
						chargeVoltage = set_voltage(current_monitor_get_bus_voltage() + 0.2);
						chargeCurrentSetpoint = 0.0;
						mahCount = 0.0;
						stepTimeCCStarting = chVTGetSystemTime();
					}
					else {
						if (ST2MS(chVTTimeElapsedSinceX(stepTimeCCStarting) > 200) {
							stepCCStartingCounter++;
							chargeCurrentSetpoint += rampUpCurrent * (ST2MS(chVTTimeElapsedSinceX(stepTimeCCStarting) / 1000));
							if (chargeCurrentSetpoint >= config->chargeCurrent) {
								chargeCurrentSetpoint = config->chargeCurrent;
								startCharging = false;
								isCharging = true;
								stepCCStartingCounter = 0;
							}
							stepTimeCCStarting = chVTGetSystemTime();
						}
					}	
				}
				
				//PID charge current
				
				currentErr = config->chargeCurrent - (-current_monitor_get_current());
				current_control_integral += currentErr * dt;
				// Windup protection
				if (current_control_integral > 0) {
					current_control_integral = 0;
				}
				else if (current_control_integral < -endOfCharge_voltage) {
					current_control_integral = -endOfCharge_voltage;
				}
				chargeVoltage = config->chargeVoltage + currentErr * config->chargeCurrentGain_P + current_control_integral * config->chargeCurrentGain_I;
				if (chargeVoltage >= endOfCharge_voltage) {
					chargeVoltage = endOfCharge_voltage; //Constant Voltage (CV) stage
				}
				set_voltage(chargeVoltage);
				break;
				
			case FULL_CURRENT:
				if (startCharging) {
					palSetPad(CHG_SW_GPIO, CHG_SW_PIN);
					startCharging = false;
					isCharging = true;
					set_voltage(endOfCharge_voltage); //TBC : Very dangerous if current is not checked!
					mahCount = 0.0;
				}
				else {
					set_voltage(endOfCharge_voltage); //TBC : Very dangerous if current is not checked!
				}
				break;
				
			case BYPASS_CC:
				if (startCharging) {
					palSetPad(CHG_SW_GPIO, CHG_SW_PIN);
					startCharging = false;
					isCharging = true;
					set_voltage(0); //TBC : Very dangerous if current is not checked!
					mahCount = 0.0;
				}
				else {
					set_voltage(0);
				}
				break;
				
			case BYPASS_CCCV:
				if (startCharging) {
					palSetPad(CHG_SW_GPIO, CHG_SW_PIN);
					startCharging = false;
					isCharging = true;
					set_voltage(0); //TBC : Very dangerous if current is not checked!
					mahCount = 0.0;
				}
				else {
					set_voltage(0);
				}
				break;
				
			default:
				charger_disable();
				break;
		}
		
		//Overccurent protection
		if (fabs(fabs(current_monitor_get_current()) >= config->maxChargeCurrent) {
			
			faults_set_fault(FAULT_OVERCURRENT);
			charger_disable();
		}
		
		//Balancing
		if (!balancing) {
			for (uint8_t i = 0; i < config->numCells; i++) {
				if (cells[i] > highestCellV) {
					highestCellV = cells[i];
				}
				if (cells[i] < lowestCellV) {
					lowestCellV = cells[i];
					lowestCellNum = i;
				}
			}
			if ((highestCellV >= config->balanceStartVoltage && highestCellV - lowestCellV > config->balanceDifferenceThreshold) && ST2MS(chVTTimeElapsedSinceX(balanceUpdateTime)) > 10000) {
				balancing = true;
				balanceUpdateTime = chVTGetSystemTime();
			}
		}
		else if (ST2MS(chVTTimeElapsedSinceX(balanceUpdateTime)) > 500) {
		//palClearPad(CHG_SW_GPIO, CHG_SW_PIN); // TBC : mandatory to disable main charging in case of balancing ?
		
			bool continueBalance = false;
			for (uint8_t i = 0; i < config->numCells; i++) {
				if (cells[i] > (cells[lowestCellNum] + 0.02) {
					ltc6803_enable_balance(i + 1);
					isBalancingCell[i]=true;
					continueBalance = true;
				}
				else (cells[i] < (cells[lowestCellNum]){
					ltc6803_disable_balance(i + 1);
					isBalancingCell[i]=false;
				}
			}
			balancing = continueBalance;
			balanceUpdateTime = chVTGetSystemTime();
		}
	}		
    // else {
        // if (!isCharging) {
            // palSetPad(CHG_SW_GPIO, CHG_SW_PIN);
            // chargeComplete = false;
            // chargeCompleteCounter = 0;
        // }
        // else {
            // palClearPad(CHG_SW_GPIO, CHG_SW_PIN);
			// current_control_integral = 0.0;
			// balancing = false;
			// ltc6803_disable_balance_all();
		// }
    // }
    lastTime = chVTGetSystemTime();
	
	//mAh count while charging
	if ((isCharging || startCharging) && (ST2MS(chVTTimeElapsedSinceX(mahCountDeltaTime)) > 100)) {
		mahCount += current_monitor_get_current() * 1000 * ST2MS(chVTTimeElapsedSinceX(mahCountDeltaTime)) /1000 /3600;
		mahCountDeltaTime = chVTGetSystemTime();
	}
	
	//End of charge detection
	if ((isCharging || startCharging) && (current_monitor_get_bus_voltage() >= endOfCharge_voltage && fabs(current_monitor_get_current()) < (config->chargeCurrent * 0.05))) { // Threshold for end of charge : 5% of rated current
		chargeCompleteCounter++;
		if (chargeCompleteCounter > 100) {
			chargeComplete = true;
			balancing = false;
			isCharging = false;
			startCharging = false;
		}
	}
	else {
		chargeCompleteCounter = 0;
	}
}

bool charger_is_charging(void) {
    return (startCharging || isCharging) && !chargeComplete;
}

bool charger_is_balancing(void) {
    return balancing;
}

float charger_get_input_voltage(void) {
    return input_voltage;
}

float charger_get_output_voltage(void) {
    return output_voltage;
}

uint16_t charger_get_mahCharged(void) {
		return (uint16_t)mahCount;
}

void charger_enable(void) {
	if (!isCharging && !chargeComplete) {
		startCharging = true;
	}
    charge_enabled = true;
	storage = false;
}

void charger_enable_storage(void) {
	if (!isCharging && !chargeComplete) {
		startCharging = true;
	}
    charge_enabled = true;
	storage = true;
}

void charger_disable(void) {
    charge_enabled = false;
	storage = false;
    palClearPad(CHG_SW_GPIO, CHG_SW_PIN);
}

bool* charger_get_balacing_cells (void) {
		return isBalancingCell;
}

static void set_voltage(float voltage) {
    i2cAcquireBus(&I2C_DEV);
#ifdef BATTMAN_4_0
    float resistance = (5100.0 * voltage - 1.26 * (5100.0 + 200000.0)) / (1.26 - voltage);
    uint16_t value = (uint16_t)((resistance / 20000.0) * 1024.0);
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = 0x01 << 2 | ((value >> 8) & 0x03);
    tx[1] = value & 0xFF;
    i2cMasterTransmitTimeout(&I2C_DEV, 0x2E, tx, 2, rx, 0, MS2ST(10));
#endif	
#elif defined(BATTMAN_4_1) || defined(BATTMAN_4_2)
#if defined(BATTMAN_4_1)
    float dac_voltage = 0.075 * (51.66 - voltage);
#endif
#else
    float dac_voltage = 0.075 * (51.584 - voltage);
#endif
    if (dac_voltage < 0.0) {
        dac_voltage = 0.0;
	}
    else if (dac_voltage > 3.3) {
        dac_voltage = 3.3;
	}
	
    uint16_t value = (uint16_t)((dac_voltage / 3.3) * 16383.0);
    uint8_t tx[3];
    uint8_t rx[2];
    tx[0] = 0x01; //Command : CODE_LOAD (14-bit mode)
    tx[1] = (uint8_t)(value >> 6);
    tx[2] = (uint8_t)((value & 0xFF) << 2);
    i2cMasterTransmitTimeout(&I2C_DEV, I2C_ADDRESS, tx, 3, rx, 0, MS2ST(10));
#endif
    output_voltage = voltage;
    i2cReleaseBus(&I2C_DEV);
}
