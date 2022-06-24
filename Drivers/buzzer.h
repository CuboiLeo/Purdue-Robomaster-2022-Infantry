#ifndef _BUZZER_H
#define _BUZZER_H
#include "sys.h"

extern void buzzer_init(uint16_t arr, uint16_t psc);
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);

#endif

