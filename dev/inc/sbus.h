#ifndef _SBUS_H_
#define _SBUS_H_

/* ----------------------- RC Channel Definition---------------------------- */
#define SBUS_CH_VALUE_MIN              364U
#define SBUS_CH_VALUE_OFFSET           1024U
#define SBUS_CH_VALUE_MAX              1684U
#define SBUS_BUFFER_SIZE               25U

#define UART_SBUS                     &UARTD4

//wheel: 				ch5  364-1684
//left switch:  ch6	 511,1024,1541
//right switch: ch7	 511,1024,1541

typedef enum{
	SBUS_S_DUMMY = 0,
	SBUS_S_UP = 1541,
	SBUS_S_DOWN = 511,
	SBUS_S_MIDDLE = 1024,
} SBUS_switch_t;

//SBUS CONFIGURATION:
//					LEFT SWITCH									RIGHT SWITCH
//UP				Flight mode SDK							Player attention
//MIDDLE		Flight mode P								Idle
//DOWN			Failsafe flight mode G			System failure

typedef enum{
	SBUS_UNCONNECTED = 0,
	SBUS_OFFLINE,
	SBUS_CONNECTED
} SBUS_state_t;

typedef struct{
	uint8_t start;
	uint16_t ch1 : 11;
	uint16_t ch2 : 11;
	uint16_t ch3 : 11;
	uint16_t ch4 : 11;
	uint16_t ch5 : 11;
	uint16_t ch6 : 11;
	uint16_t ch7 : 11;
	uint16_t ch8 : 11;
	uint16_t ch9 : 11;
	uint16_t ch10 : 11;
	uint16_t ch11 : 11;
	uint16_t ch12 : 11;
	uint16_t ch13 : 11;
	uint16_t ch14 : 11;
	uint16_t ch15 : 11;
	uint16_t ch16 : 11;
	uint8_t  ch17     	:1;
	uint8_t  ch18 		  :1;
	uint8_t	 frame_lost :1;
	uint8_t	 failsafe		:1;
	uint8_t	 shit				:4; //Useless
	uint8_t	end;
}__attribute__((packed)) SBUS_t;

SBUS_state_t SBUS_getState(void);
SBUS_t* SBUS_get(void);
void SBUS_init(void);

#endif
