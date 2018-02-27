#include "config.h"
#include "ch.h"
#include "hal.h"
#include "eeprom.h"
#include "stm32f30x_conf.h"
#include <string.h>
#include "utils.h"
#include <stddef.h>
#include "current_monitor.h"

#define EEPROM_BASE              1000

// Global variables
uint16_t VirtAddVarTab[NB_OF_VAR];

static volatile Config config;

void config_init(void)
{
    memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

    int ind = 0;
    for (unsigned int i = 0; i < (sizeof(Config) / 2); i++) {
	VirtAddVarTab[ind++] = EEPROM_BASE + i;
    }

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
    EE_Init();
    config_read_all();
	
	//Checking the validity of certain parameters
}

void config_load_default_configuration(void)
{
    config.CANDeviceID = 0x09;
    config.numCells = 12;
    config.fullCellVoltage = 4.2;
    config.emptyCellVoltage = 3.4;
    config.packCapacity = 2500;
    config.cellLowVoltageCutoff = 3.2;
    config.cellLowVoltageWarning = 3.4;
    config.cellHighVoltageCutoff = 4.25;
    config.cellHighVoltageWarning = 4.21;
	config.cellEndChargeVoltage = 4.2;
	config.cellStorageVoltage = 3.85;
    config.maxCurrentCutoff = 120.0;
    config.maxContinuousCurrent = 100.0;
    config.continuousCurrentCutoffTime = 30;
    config.continuousCurrentCutoffWarning = 10;
    config.maxChargeCurrent = 20.0;
    //config.chargeVoltage = 25.2;
    config.chargeCurrent = 2.0;
    config.turnOnDelay = 200;
    config.shutdownDelay = 500;
    config.chargeMode = FULL_CURRENT;
    config.chargeCurrentGain_P = 0.1;
    config.chargeCurrentGain_I = 1.0;
    config.prechargeTimeout = 500;
    config.balanceStartVoltage = 3.5;
    config.balanceDifferenceThreshold = 0.01;
	config.tempBoardWarning = 80.0;
	config.tempBoardCutoff = 100.0;
	config.tempBattWarning = 50.0;
	config.tempBattCutoff = 70.0;
	config.sleepModeTime = 60;
	config.masterVescCanID = 0;
	config.chargerDisconnectShutdown = true;
	config.isBattTempSensor = false;
	config.enBuzzer = true;
	config.enSleepMode = false;
	config.enVescCanComm = true;
}

Config* config_get_configuration(void)
{
    return &config;
}

bool config_write_all(void)
{
    utils_sys_lock_cnt();

    bool is_ok = true;
    uint8_t *conf_addr = (uint8_t*)&config;
    uint16_t var;

    FLASH_ClearFlag(FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    for (unsigned int i = 0; i < (sizeof(Config) / 2); i++) {
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;

		if (EE_WriteVariable(EEPROM_BASE + i, var) != FLASH_COMPLETE) {
			is_ok = false;
			break;
		}
    }

    utils_sys_unlock_cnt();
    return is_ok;
}

bool config_write_field(uint16_t addr, uint8_t *data, uint8_t size) {
	
    if (addr == offsetof(Config, CANDeviceID)) {

    }
    else if (addr == offsetof(Config, numCells)) {
        if (*data > 12) {
            *data = 12;
        }
    }
    else if (addr == offsetof(Config, cellLowVoltageCutoff)) {
		if (*((float*)data) < 0.0) {
            *((float*)data) = 0.0;
        }
		else if (*((float*)data) > config.cellHighVoltageCutoff) {
			*((float*)data) = config.cellHighVoltageCutoff;
		}
    }
    else if (addr == offsetof(Config, cellHighVoltageCutoff)) {
        if (*((float*)data) > 4.5) {
            *((float*)data) = 4.5;
        }
        else if (*((float*)data) < 0.0) {
            *((float*)data) = 0.0;
        }
		else if (*((float*)data) < config.cellLowVoltageCutoff) {
			*((float*)data) = config.cellLowVoltageCutoff;
		}
    }
    else if (addr == offsetof(Config, chargeVoltage)) {
        if (*((float*)data) > config.numCells * config.cellHighVoltageCutoff) {
            *((float*)data) = config.numCells * config.cellHighVoltageCutoff;
        }
        else if (*((float*)data) < 0.0) {
            *((float*)data) = 0.0;
        }
    }
    else if (addr == offsetof(Config, maxCurrentCutoff)) {
        if (*((float*)data) > 150.0) {
            *((float*)data) = 150.0;
        }
        else if (*((float*)data) < 1.0) {
            *((float*)data) = 1.0;
        }
        current_monitor_set_overcurrent(*((float*)data));
    }
	else if (addr == offsetof(Config, maxContinuousCurrent)) {
        if (*((float*)data) > Config.maxCurrentCutoff) {
            *((float*)data) = Config.maxCurrentCutoff);
        }
        else if (*((float*)data) < 1.0) {
            *((float*)data) = 1.0;
        }
    }
	else if (addr == offsetof(Config, chargeCurrent)) {
        if (*((float*)data) > Config.maxChargeCurrent) {
            *((float*)data) = Config.maxChargeCurrent);
        }
        else if (*((float*)data) < 1.0) {
            *((float*)data) = 1.0;
        }
    }
    else {
        /*return false;*/
    }
    memcpy((void*)&config + addr, (void*)data, size);

    utils_sys_lock_cnt();

    bool is_ok = true;
    uint8_t *conf_addr = (uint8_t*)&config;
    uint16_t var;

    FLASH_ClearFlag(FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    for (unsigned int i = (addr % 2 == 0 ? addr : addr - 1); i < addr + size; i += 2) {
        var = (conf_addr[i] << 8) & 0xFF00;
        var |= conf_addr[i + 1] & 0xFF;

        if (EE_WriteVariable(EEPROM_BASE + i / 2, var) != FLASH_COMPLETE) {
            is_ok = false;
            break;
        }
    }

    utils_sys_unlock_cnt();
    return is_ok;
}

void config_read_all(void) {
    bool is_ok = true;
    uint8_t *conf_addr = (uint8_t*)&config;
    uint16_t var;

    for (unsigned int i = 0; i < (sizeof(Config) / 2); i++) {
		if (EE_ReadVariable(EEPROM_BASE + i, &var) == 0) {
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		} 
		else {
			is_ok = false;
			break;
		}
    }

    if (!is_ok) {
        config_load_default_configuration();
        config_write_all();
    }
}