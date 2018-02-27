#include "faults.h"
#include "datatypes.h"

static uint8_t faults = FAULT_NONE;
static uint16_t warnings = WARNING_NONE;

void faults_set_fault(Fault fault)
{
    faults |= fault;
}

uint8_t faults_get_faults(void)
{
    return faults;
}

void faults_clear_fault(Fault fault)
{
    faults &= ~fault;
}

void faults_clear_all_faults(void)
{
    faults = FAULT_NONE;
}

bool faults_check_fault(Fault fault)
{
    return faults & fault;
}

void faults_set_warning(Warning warning)
{
    warnings |= warning;
}

uint16_t faults_get_warnings(void)
{
    return warnings;
}

void faults_clear_warning(Warning warning)
{
    warnings &= ~warning;
}

void faults_clear_all_warnings(void)
{
    warnings = WARNING_NONE;
}

bool faults_check_warning(Warning warning)
{
    return warnings & warning;
}

//TODO
/* 
void faults_values_snapshot(void) { //TODO Snapshot of values when fault trigerred
	float busVoltage;
	float current;
	float batteryTemp;
	float ltc6803Temp;
	float boardTemp;

		Fault fault_code;
		
    float busVoltage;
	float current;
	float batteryTemp;
	float boardTemp;
	bool isDischarging;
	bool isCharging;
	uint8_t second_fault;
	uint8_t minute_fault;
	uint8_t hour_fault;
	uint8_t day_fault;
	uint8_t month_fault;
	uint8_t year_fault,
}
 */