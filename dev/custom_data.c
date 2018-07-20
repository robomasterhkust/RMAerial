#include "ch.h"
#include "hal.h"

#include "custom_data.h"
#include "judge.h"

#include <stdbool.h>
#include "string.h"

static Custom_Data_t customData;
static size_t sizeout;

static judge_fb_t* pJudge;

static uint8_t  led_stream;
static uint8_t  ledStream_dir;
static uint16_t ledStream_speed;

#define  CUSTOM_DATA_UPDATE_PERIOD_MS      50U // the update frequency is 100ms

void customData_put(const uint8_t pos, const uint8_t value)
{
	customData.data[pos] = value;
}

void customData_setLED(const uint8_t value)
{
	customData.stream = value;
}

void customData_setIndicator(const uint8_t pos, const uint8_t value)
{
	if(pos == LED_INDICATOR_MESSAGE)
		customData.indicator1 = value ? 0 : 1;
	if(pos == LED_INDICATOR_OSDK)
		customData.indicator2 = value ? 0 : 1;
}

void customData_setLEDStream(const led_stream_dir_t dir, const uint8_t freq)
{
	ledStream_dir = dir;
	if(dir == LEDS_STATIC)
		return;
	else
		ledStream_speed = 1000 * freq / CUSTOM_DATA_UPDATE_PERIOD_MS;
}

#define LED_LEFT_MASK   0b100000
#define LED_RIGHT_MASK	0b000001
static THD_WORKING_AREA(custom_data_thread_wa, 1024);
static THD_FUNCTION(custom_data_thread, p)
{
   Custom_Data_t *d = (Custom_Data_t *) p;
   chRegSetThreadName("Update Custom Data");

	 uint32_t count = 0;
   while (!chThdShouldTerminateX())
	 {
		 count++;

		 if(led_stream == LED_STREAM_VALUE)
		 {
			 if(!(count % ledStream_speed))
			 {
				 if(ledStream_dir == LEDS_STREAM_LEFT)
				 {
					 uint8_t prev_led_stream    = d->stream;
					 uint8_t left = (d->stream & LED_LEFT_MASK) >> 6;
					 d->stream = (prev_led_stream >> 1) | left;
				 }
				 else if(ledStream_dir == LEDS_STREAM_RIGHT)
				 {
					 uint8_t prev_led_stream    = d->stream;
					 uint8_t right = (d->stream & LED_RIGHT_MASK);
					 d->stream = (prev_led_stream >> 1) | right;
				 }
			 }
		 }
		 else
		 	d->stream = led_stream;

  	 sizeout = judgeDataWrite(d->data[0], d->data[1], d->data[2], d->indicator1 << 7 |
		 																															d->stream << 1 		|
																																	d->indicator2);
  	 chThdSleepMilliseconds(CUSTOM_DATA_UPDATE_PERIOD_MS);
   }
}

void customData_init(void)
{
  pJudge = judgeDataGet();

	memset(&customData, 0, sizeof(Custom_Data_t));

	customData.stream = LED_STREAM_VALUE;
  chThdCreateStatic(custom_data_thread_wa, sizeof(custom_data_thread_wa),
                 NORMALPRIO + 7,
                 custom_data_thread, &customData);
}
