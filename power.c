#include "power.h"
#include "hw_conf.h"
#include "analog.h"
#include "current_monitor.h"
#include "config.h"
#include "charger.h"
#include "rtcc.h"
#include "faults.h"
#include "comm_can.h"

static volatile bool discharge_enabled = false;
static volatile bool precharged = false;
static volatile bool isPrecharging = false;
static volatile systime_t prechargeStartTime;
static volatile PowerOnEvent power_on_event;
static volatile PowerStatus power_status;
static volatile bool power_button_prev = false;
static volatile bool shutdownOrder;
static volatile bool shutdownStarted = false;
static volatile bool shutdown = false;
static volatile systime_t shutdownStartTime;
static volatile systime_t sleepModeTimer;
static volatile systime_t btnLongPressTimer;
static volatile Config *config;
static volatile bool memLongPress = false;

void power_init(void)
{
    config = config_get_configuration();
	
    palClearPad(DSG_SW_GPIO, DSG_SW_PIN);
    palClearPad(PCHG_SW_GPIO, PCHG_SW_PIN);
	power_status = STANDBY;
	
    if (!palReadPad(PWR_BTN_GPIO, PWR_BTN_PIN)) {
        power_on_event = EVENT_SWITCH;
	}
    else if (analog_charger_input_voltage() > 6.0) {
        power_on_event = EVENT_CHARGER;
	}
#if defined(BATTMAN_4_1) || defined(BATTMAN_4_2)
    else if (!palReadPad(RTCC_INT_GPIO, RTCC_INT_PIN)) {
	    power_on_event = EVENT_RTCC;
	}
#endif
#if defined(BATTMAN_4_2)
    else if (palReadPad(USB_DETECT_GPIO, USB_DETECT_PIN)){
        power_on_event = EVENT_USB;
    }
#endif

    if (power_on_event == EVENT_SWITCH){
        chThdSleepMilliseconds(config->turnOnDelay);
        buzzer_play_note(50,500);
        discharge_enabled = false;
    }
    if (power_on_event != EVENT_USB) {
        palSetPad(PWR_SW_GPIO, PWR_SW_PIN);
	}
}

void power_update(void) {

//     if (!shutdown && power_on_event != EVENT_USB) {
//         palSetPad(PWR_SW_GPIO, PWR_SW_PIN);
//     }

//INPUT READINGS
bool btnShortPress = false;
bool btnLongPress = false;

	//Button (differencing short and long press
	if (palReadPad(PWR_BTN_GPIO, PWR_BTN_PIN) {
		if (!memLongPress && ST2MS(chVTTimeElapsedSinceX(btnLongPressTimer) > 3000)) { //long press delay
			btnLongPress = true;
			memLongPress = true;
		}
	}
	else {
		if (btnPreviousState && !memLongPress) { //Falling edge of button
			btnShortPress = true;
		}
		btnLongPressTimer = chVTGetSystemTime();
		memLongPress = false;
	}
	btnPreviousState = palReadPad(PWR_BTN_GPIO, PWR_BTN_PIN);
	

//DISCHARGE MANAGEMENT

	if (charger_is_charging()) {
		power_status = CHARGING;
	}
	else (power_status == CHARGING) {
		power_status = STANDBY;
	}
	
	if (faults_get_faults() != FAULT_NONE) { //Check the fault presence
		power_switchOff();
		discharge_enabled=false;
	}
	else if (power_on_event == EVENT_CHARGER || analog_charger_input_voltage() > 6.0) {
		power_switchOff();
		discharge_enabled=false;
		charger_enable();
	}
	else {
		if (discharge_enabled) {
			if (!precharged){
				if (!isPrecharging) {
					prechargeStartTime = chVTGetSystemTime();
					palSetPad(PCHG_SW_GPIO, PCHG_SW_PIN);
					isPrecharging=true;
					power_status = PRECHARGING;
				}
				if (isPrecharging && ST2MS(chVTTimeElapsedSinceX(prechargeStartTime)) > 8){
					if (analog_discharge_voltage() < 0.5){ // Turned on into short
						palClearPad(DSG_SW_GPIO, DSG_SW_PIN);
						palClearPad(PCHG_SW_GPIO, PCHG_SW_PIN);
						faults_set_fault(FAULT_TURN_ON_SHORT);
						power_status = STANDBY;
						return;
					}
				}
				if (analog_discharge_voltage() >= current_monitor_get_bus_voltage() - 5.0 || ST2MS(chVTTimeElapsedSinceX(prechargeStartTime)) > config->prechargeTimeout) {
					precharged = true;
					isPrecharging = false;
				}
			}
			if (precharged) {
				palSetPad(DSG_SW_GPIO, DSG_SW_PIN);
				palClearPad(PCHG_SW_GPIO, PCHG_SW_PIN);
				power_status = DISCHARGING;				
			}
			/*else
			{
				palSetPad(PCHG_SW_GPIO, PCHG_SW_PIN);
				palClearPad(DSG_SW_GPIO, DSG_SW_PIN);
				power_status = PRECHARGING;
			} */
		}
		else
		{
			power_switchOff();
			precharged = false;
		}
	}
	
	//Sleep mode
	if (current_monitor_get_current() < -0.5 || current_monitor_get_current() > 0.5) {
		sleepModeTimer = chVTGetSystemTime();
	}
	if (enSleepMode && power_status == DISCHARGING && (ST2MS(chVTTimeElapsedSinceX(sleepModeTimer)) > (config->sleepModeDelay * 60000.0)) {
		enterSleepMode = true;
	}
	
	//Button management
	if (power_status == CHARGING) {
		if (btnLongPress) {
			charger_enable_storage();
		}
	else if (btnShortPress || enterSleepMode)) {
		shutdownOrder = true;
	}
	
	
//SHUTDOWN MANAGEMENT

    if (shutdownOrder || (config->chargerDisconnectShutdown && power_on_event == EVENT_CHARGER && analog_charger_input_voltage() < 6.0))
    {
        shutdown = true; 
    }
    else if ((analog_charger_input_voltage() < 6.0 && !palReadPad(USB_DETECT_GPIO, USB_DETECT_PIN)) || power_on_event == EVENT_RTCC)
    {
        if (!shutdownStarted)
        {
            shutdownStartTime = chVTGetSystemTime();
			power_switchOff();
            shutdownStarted = true;
        }
        if (ST2MS(chVTTimeElapsedSinceX(shutdownStartTime)) > config->shutdownDelay)
        {
            shutdown = true;
        }
    }
    else
    {
        shutdown = false;
        shutdownStarted = false;
    }
	
	if (shutdown) {
        power_switchOff();
		rtcc_enable_alarm();
		palClearPad(PWR_SW_GPIO, PWR_SW_PIN);
		//If flash memory log implemented, wait for the end of writing before release PWR_SW
	}
	
	
	
	/*
    if (shutdown || (config->chargerDisconnectShutdown && power_on_event == EVENT_CHARGER && analog_charger_input_voltage() < 6.0))
    {
        shutdown = true;
        rtcc_enable_alarm();
        powerSwitchOff();
        palClearPad(PWR_SW_GPIO, PWR_SW_PIN);
		//If flash memory log implemented, wait for the end of writing before release PWR_SW
    }
    else if (((!palReadPad(PWR_BTN_GPIO, PWR_BTN_PIN) && power_button_released) && analog_charger_input_voltage() < 6.0 && !palReadPad(USB_DETECT_GPIO, USB_DETECT_PIN)) || power_on_event == EVENT_RTCC)
    {
        if (!shutdownStarted)
        {
            shutdownStartTime = chVTGetSystemTime();
			power_switchOff();
            shutdownStarted = true;
        }
        if (ST2MS(chVTTimeElapsedSinceX(shutdownStartTime)) > config->shutdownDelay)
        {
            rtcc_enable_alarm();
            palClearPad(PWR_SW_GPIO, PWR_SW_PIN);
            shutdown = true;
        }
    }
    else
    {
        shutdown = false;
        shutdownStarted = false;
    }
	*/
}

void power_switchOff(void){
	
	if (power_status == STANDBY || power_status == CHARGING) {
		return;
	}
	else {
		if (config->enVescCanComm) {
			comm_can_set_current(config->masterVescCanID, 0.0);
		}
		palClearPad(DSG_SW_GPIO, DSG_SW_PIN);
		palClearPad(PCHG_SW_GPIO, PCHG_SW_PIN);
		power_status = STANDBY;
		precharged = false;
		discharge_enabled = false;
	}
}

void power_enable_discharge(void)
{
    discharge_enabled = true;
}

// void power_disable_discharge(void)
// {
    // discharge_enabled = false;
    // palClearPad(DSG_SW_GPIO, DSG_SW_PIN);
	
// }

void power_set_shutdown(void)
{
    shutdownOrder = true;
}

bool power_is_shutdown(void)
{
    return shutdown;
}

PowerOnEvent power_get_power_on_event(void)
{
    return power_on_event;
}

PowerStatus power_get_status(void)
{
    return power_status;
}
