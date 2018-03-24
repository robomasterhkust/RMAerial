#ifndef _SHOOT_H_
#define _SHOOT_H_

#define SHOOTER_USE_RC

#ifdef __cplusplus
extern "C" {
#endif

void shooter_control(uint16_t setpoint);
void shooter_init(void);

#ifdef __cplusplus
}
#endif


#endif
