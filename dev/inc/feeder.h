#ifndef FEEDER
#define FEEDER

#define FEEDER_SINGLE_TIMEOUT_MS 100U

#define FEEDER_CAN          &CAND1
#define FEEDER_CAN_EID       0x1FF

#define FEEDER_BULLET_PER_TURN    7U
#define FEEDER_GEAR              36U
#define FEEDER_SET_RPS           19U     //Rounds per second of feeder

#define FEEDER_OUTPUT_MAX       10000U
#define FEEDER_OUTPUT_MAX_BACK   6000U  //output limit for stuck-bullet turning back
#define FEEDER_ERROR_COUNT        100U


//#define FEEDER_USE_BOOST              //Very Violent!!
#ifdef FEEDER_USE_BOOST
  #define FEEDER_BOOST_POWER 10000U
#endif

typedef enum{
  FEEDER_STOP = 0,
  FEEDER_SINGLE,    //Single shot
  FEEDER_AUTO,      //Auto fire
  FEEDER_FINISHED,  //Finished a round of shooting
  SAVE_LIFE, //OVER HEAT!
  #ifdef FEEDER_USE_BOOST
    FEEDER_BOOST
  #endif //FEEDER_USE_BOOST
}feeder_mode_t;

typedef struct{
    float kp;
    float ki;
    float kd;
    float inte_max;
    float inte;
} __attribute__((packed)) pid_struct;

bool feeder_get_error(void);
void feeder_clear_error(void);
int16_t feeder_canUpdate(void); //In case the feeder ESC is using the same EID as gimbal does
void feeder_bulletOut(void);  //Used as limit switch EXTI funtion
void feeder_singleShot(void); //Rune shooting function

void feeder_init(void);
void feeder_start(void);

#endif
