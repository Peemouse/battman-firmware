#include "power.h"
#include "hw_conf.h"
#include "analog.h"
#include "current_monitor.h"
#include "config.h"
#include "charger.h"
#include "rtcc.h"
#include "faults.h"

static volatile bool discharge_enabled = false;
static volatile bool precharged = false;
static volatile bool isPrecharging = false;
static volatile systime_t prechargeStartTime;
static volatile PowerOnEvent power_on_event;
static volatile PowerStatus power_status;
static volatile bool power_button_released = false;
static volatile bool shutdownOrder;
static volatile bool shutdownStarted = false;
static volatile bool shutdown = false;
static volatile systime_t shutdownStartTime;
static volatile Config *config;

static void powerSwitchOff(void);


void power_init(void)
{
    config = config_get_configuration();
	
    palClearPad(DSG_SW_GPIO, DSG_SW_PIN);
    palClearPad(PCHG_SW_GPIO, PCHG_SW_PIN);
	power_status = STANDBY;
	
    if (!palReadPad(PWR_BTN_GPIO, PWR_BTN_PIN))
        power_on_event = EVENT_SWITCH;
    else if (analog_charger_input_voltage() > 6.0)
        power_on_event = EVENT_CHARGER;
	
#if defined(BATTMAN_4_1) || defined(BATTMAN_4_2)
    else if (!palReadPad(RTCC_INT_GPIO, RTCC_INT_PIN))
        power_on_event = EVENT_RTCC;
#endif
#if defined(BATTMAN_4_2)
    else if (palReadPad(USB_DETECT_GPIO, USB_DETECT_PIN))
    {
        power_on_event = EVENT_USB;
    }
#endif

    if (power_on_event == EVENT_SWITCH)
    {
        chThdSleepMilliseconds(config->turnOnDelay);
        discharge_enabled = true;
    }
    if (power_on_event != EVENT_USB)
        palSetPad(PWR_SW_GPIO, PWR_SW_PIN);
}

void power_update(void) {

    if (!shutdown && power_on_event != EVENT_USB)
        palSetPad(PWR_SW_GPIO, PWR_SW_PIN);

//DISCHARGE MANAGEMENT
    if (discharge_enabled && faults_get_faults() == FAULT_NONE) {
		if (faults_get_faults() != FAULT_NONE) //Check again the fault presence to be sure
		{
			powerSwitchOff();
			return;
		}
		else {
			if (!precharged)
			{
				if (!isPrecharging) {
					prechargeStartTime = chVTGetSystemTime();
					palSetPad(PCHG_SW_GPIO, PCHG_SW_PIN);
					isPrecharging=true;
					power_status = PRECHARGING;
				}
				if (isPrecharging && ST2MS(chVTTimeElapsedSinceX(prechargeStartTime)) > 8)
				{
					if (analog_discharge_voltage() < 0.5) // Turned on into short
					{
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
	}
    else
    {
		powerSwitchOff();
        precharged = false;
    }
	
//SHUTDOWN MANAGEMENT
    if (shutdownOrder || (config->chargerDisconnectShutdown && power_on_event == EVENT_CHARGER && analog_charger_input_voltage() < 6.0))
    {
        shutdown = true;
    }
    else if (((!palReadPad(PWR_BTN_GPIO, PWR_BTN_PIN) && power_button_released) && analog_charger_input_voltage() < 6.0 && !palReadPad(USB_DETECT_GPIO, USB_DETECT_PIN)) || power_on_event == EVENT_RTCC)
    {
        if (!shutdownStarted)
        {
            shutdownStartTime = chVTGetSystemTime();
			powerSwitchOff();
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
        powerSwitchOff();
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
			powerSwitchOff();
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
//TODO : clean command of shutdown

void powerSwitchOff(void){
	palClearPad(DSG_SW_GPIO, DSG_SW_PIN);
    palClearPad(PCHG_SW_GPIO, PCHG_SW_PIN);
	power_status = STANDBY;
}

void power_enable_discharge(void)
{
    discharge_enabled = true;
}

void power_disable_discharge(void)
{
    discharge_enabled = false;
    palClearPad(DSG_SW_GPIO, DSG_SW_PIN);
}

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
