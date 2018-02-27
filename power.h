#ifndef _POWER_H_
#define _POWER_H_

#include "datatypes.h"

void power_init(void);
void power_update(void);
void power_enable_discharge(void);
void power_switchOff(void);
void power_set_shutdown(void);
bool power_is_shutdown(void);
PowerOnEvent power_get_power_on_event(void);
PowerStatus power_get_status(void);

#endif /* _POWER_H_ */
