#include "temp.h"
#include "analog.h"
#include "ltc6803.h"
#include "config.h"
#include "faults.h"

static volatile Config *config;

void temp_init(void)
{
    config = config_get_configuration();
	
}

void temp_update(void) {
	float boardTemp;
	float* ltc6803Temp; //[0]=battery temp, [1]=not used, [2]=Internal LTC6803 temp
		
	boardTemp = analog_temperature();
	ltc6803Temp = ltc6803_get_temp();

//TRIGGERING FAULTS & WARNINGS
	//Over LTC6803 internal temperature (analog and THSD bit)

	if (ltc6803Temp[2] >= 85.0) { //TODO : add TSHD check
		faults_set_warning(WARNING_LTC6803_TEMP);
	}
	
	//Over board temperature
	if (boardTemp >= config->tempBoardCutoff) {
		faults_set_fault(FAULT_BOARD_TEMP);
	}
	else {
		if (boardTemp >= config->tempBoardWarning) {
			faults_set_warning(WARNING_BOARD_TEMP);
		}
	}
		
	//Over battery temperature (if sensor connected)
	if (config->isBattTempSensor) {
		if (ltc6803Temp[0] >= config->tempBattCutoff) {
			faults_set_fault(FAULT_BATTERY_TEMP);
		}
		else if (ltc6803Temp[0] >= config->tempBattWarning) {
				faults_set_warning(WARNING_BATTERY_TEMP);
			}
		else {
			faults_clear_fault(FAULT_BATTERY_TEMP);
			faults_clear_warning(WARNING_BATTERY_TEMP);
		}
	}

//CLEARING FAULTS & WARNINGS
	//LTC6803 internal temperature OK (analog and THSD bit)

	if (ltc6803Temp[2] < 65.0) {
		faults_clear_warning(WARNING_LTC6803_TEMP);
	}
	
	//Board temperature ok
	if (boardTemp < (config->tempBoardWarning - 10.0)) {
		faults_clear_warning(WARNING_BOARD_TEMP);
		faults_clear_fault(FAULT_BOARD_TEMP);
	}
	else if (boardTemp < config->tempBoardWarning) {
			faults_clear_fault(FAULT_BOARD_TEMP);
	}
		
	//Battery temperature (if sensor connected)
	
	if (config->isBattTempSensor) {
		if (ltc6803Temp[0] < config->tempBattWarning - 10.0) {
			faults_clear_warning(WARNING_BATTERY_TEMP);
			faults_clear_fault(FAULT_BATTERY_TEMP);
		}
		else if (ltc6803Temp[0] < config->tempBattWarning) {
				faults_clear_fault(FAULT_BATTERY_TEMP);
		}
	}
}
		


