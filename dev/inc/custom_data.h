#ifndef CUSTOM_DATA_H_
#define CUSTOM_DATA_H_

typedef enum
{
	LED_INDICATOR_OSDK = 0,
	LED_INDICATOR_MESSAGE = 1,
} led_indicator_t;

typedef enum
{
	LEDS_STATIC = 0,
	LEDS_STREAM_LEFT,
	LEDS_STREAM_RIGHT
} led_stream_dir_t;

typedef struct{
	float data[3];
	uint8_t indicator1 : 1;
	uint8_t stream     : 6;
	uint8_t indicator2 : 1;
} __attribute__((packed)) Custom_Data_t;

#define LED_STREAM_VALUE 0b001100

#define LED_YES 0b111111
#define LED_NO  0b000000

void customData_put(const uint8_t pos, const uint8_t value);
void customData_setLED(const uint8_t value);
void customData_setIndicator(const uint8_t pos, const uint8_t value);
void customData_setLEDStream(const led_stream_dir_t dir, const uint8_t freq);

void customData_init(void);

#endif
