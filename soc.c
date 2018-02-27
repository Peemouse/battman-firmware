#include "soc.h"
#include "current_monitor.h"
#include "config.h"
#include "power.h"
#include "ltc6803.h"

static volatile Config *config;
static float coulomb_count;
static systime_t prevTime;
float avgCellIntResistance;
float battIntResistance;
float maxVoltageSag[2];//[1]=voltage sag; [1]=current when it happened
float idleVoltage;

void soc_init(void)
{
    config = config_get_configuration();
    prevTime = chVTGetSystemTime();
	avgCellIntResistance = 0;
	battIntResistance = 0;
	avgCellIntResistance = 0;
}

void soc_update(void)
{
    float dt = ST2US(chVTTimeElapsedSinceX(prevTime)) / 1.0e6;
    prevTime = chVTGetSystemTime();
    float current = current_monitor_get_current();
	float battVoltage = current_monitor_get_bus_voltage();
	float* cells = ltc6803_get_cell_voltages();
	float sumCellsVoltage = 0;
	float voltageSag;
	
	
    coulomb_count -= current * dt;

//DELTA U : difference between sum of all cellls and main voltage measured
	for (uint8_t i = 0; i< config->numCells; i++) {
		sumCellsVoltage += cells[i];
	}
	if ((sumCellsVoltage - battVoltage) < 0.0) {
		float deltaU = battVoltage - sumCellsVoltage;
		if (deltaU > 2.0) {
			//TODO : battery voltage inconsistency
		}
	}

	
//UNDER VOLTAGE	
	if (battVoltage < config->cellLowVoltageWarning) {
		faults_set_warning(WARNING_BATTERY_UV);
	}
	if (battVoltage < config->cellLowVoltageCutoff) {
		power_switchOff();
		faults_set_fault(FAULT_BATTERY_UV);
		power_set_shutdown();
	}
	
//OVER VOLTAGE	
	if (battVoltage > config->cellHighVoltageWarning) {
		faults_set_warning(WARNING_BATTERY_OV);
	}
	if (battVoltage > (config->cellHighVoltageCutoff * config->numCells)) {
		power_switchOff();
		faults_set_fault(FAULT_BATTERY_OV);
		power_set_shutdown();
	}
	for(uint8_t i=0;i < config->numCells;i++) {
		if (cells[i] < config->emptyCellVoltage) {
			power_switchOff();
			faults_set_fault(FAULT_CELL_UV);
			power_set_shutdown();
		}
		else if (cells[i] > config->cellHighVoltageCutoff) {
			power_switchOff();
			faults_set_fault(FAULT_CELL_OV);
		}
	}
	
//INTERNAL RESISTANCE (two-tier DC load method) & VOLTAGE SAG
	
	if (current > 1.0 && current < 3.0) {
		idleVoltage = battVoltage;
	}
	else if (current > 10.0) {
		voltageSag = idleVoltage - battVoltage;
		battIntResistance = (voltageSag) / current * 1000.0;
		avgCellIntResistance = (uint8_t)(battIntResistance / config->numCells);
		if (voltageSag > maxVoltageSag[0]) {
			maxVoltageSag[0] = voltageSag;
			maxVoltageSag[1] = current;
		}
	}


	//TODO : log values to estimate battery health
}

float soc_get_coulomb_count(void)
{
    return coulomb_count;
}

float soc_get_relative_soc(void)
{
    return coulomb_count * 3600 * 1000 / config->packCapacity;
}

float soc_get_battery_IR(void) //TODO : add also avgCellIntResistance ?
{
	return battIntResistance;
}
