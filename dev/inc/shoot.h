#ifndef _SHOOT_H_
#define _SHOOT_H_

#define SHOOTER_USE_RC
// #define SHOOTER_SETUP
// void pwm12_setWidth(uint16_t width);

#define SHOOTER_LOW_SPEED  10
#define SHOOTER_HIGH_SPEED 20

void shooter_setRuneState(const uint8_t enable); //User set shooter to rune mode
uint8_t shooter_getSpeed(void);
void shooter_start(void);

typedef struct
{
	uint8_t rune_speed;
	uint8_t fast_speed;
	uint8_t slow_speed;
	uint8_t stop;
}speed_mode_t;

#endif
