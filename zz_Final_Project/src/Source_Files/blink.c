/***************************************************************************//**
 * @file
 * @brief Blink examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

//#include "sl_simple_led.h"
//#include "sl_simple_led_instances.h"
#include "os.h"
#include "blink.h"
#include "queue.h"
#include "glib.h"
#include "dmd.h"
#include <math.h>
#include <string.h>
#include "sl_board_control.h"
#include "sl_board_control_config.h"

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

// -------------- Make an enum for event flags --------- //
enum messages {LED0_ONLY = 1, LED1_ONLY = 2, LED_ALL = 3, LED_NONE = 4};
enum btn_stuff{
  BTN0_PRESS = 1,
  BTN1_PRESS = 2
};

// -------------------------------------------------------------------------- //
// ---------------- NEW DATA STRUCTURES FOR FINAL PROJECT ------------------- //
// -------------------------------------------------------------------------- //
struct shield_data shield = { .xPos = SHIELD_INITIAL_X,
                              .yPos = SHIELD_INITIAL_Y,
                              .xVel = 0,
                              .xAccl = 0,
                              .forceApplied = 0,
                              .enhanced = 0,
                              .timeEnhanced = 0,
                              .timeUnenhanced = 0
                            };

struct holtzmann_data holtzmann = { .xAccl = 0,
                                    .xForce = 0,
                                    .xPos = HM_INITIAL_X_POSITION,
                                    .xVel = HM_INITIAL_X_VELOCITY,
                                    .yAccl = 0,
                                    .yForce = GRAVITY*HM_MASS*(1/1000), // (kg*mm/s^2)*(1 m / 1000 mm) -> N = kg*m/s^2
                                    .yPos = HM_INITIAL_Y_POSITION,
                                    .yVel = 0
                                   };

struct game_data game = { .game_over = 0,
                          .laser_active = 0,
                          .num_HM_left = HM_NUM,
                          .num_laser_left = NUM_LASERS
                        };

// ---------------- FINAL PROJECT TASKS ------------------------------------- //
// LCD Display Task
static OS_TCB lcd_tcb;
static CPU_STK lcd_stack[LCD_TASK_STACK_SIZE];
// LaserUpdateTask
static OS_TCB laser_tcb;
static CPU_STK laser_stack[LASER_TASK_STACK_SIZE];
// ShieldEnhanceTask
static OS_TCB shield_enhance_tcb;
static CPU_STK shield_enhance_stack[SHIELD_ENHANCE_TASK_STACK_SIZE];
// ShieldForceTask
static OS_TCB shield_force_tcb;
static CPU_STK shield_force_stack[SHIELD_FORCE_TASK_STACK_SIZE];
// PhysicsUpdateTask
static OS_TCB physics_tcb;
static CPU_STK physics_stack[PHYSICS_TASK_STACK_SIZE];
// LED0OutputTask
static OS_TCB led0_tcb;
static CPU_STK led0_stack[LED0_TASK_STACK_SIZE];
// LED1OutputTask
static OS_TCB led1_tcb;
static CPU_STK led1_stack[LED1_TASK_STACK_SIZE];

// ----------------- FINAL PROJECT DATA STRUCTURES -------------------------- //
// BTN0 Flag Group
static OS_FLAG_GRP btn0_flag_group;
// BTN1 Flag Group
static OS_FLAG_GRP btn1_flag_group;
// ShieldData Mutex
static OS_MUTEX shield_mutex;
// HoltzmannData Mutex
static OS_MUTEX holtzmann_mutex;
// GameData Mutex
static OS_MUTEX game_mutex;
// LED0 Flag Group
static OS_FLAG_GRP led0_flag_group;
// LED1 Flag Group
static OS_FLAG_GRP led1_flag_group;

static OS_TMR led1_period_timer;
static OS_TMR led1_oneshot_timer;

static OS_TMR led0_period_timer;
static OS_TMR led0_oneshot_timer;

static OS_TMR led3_period_timer;
static OS_TMR led3_oneshot_timer;


enum direction_vals {STRAIGHT = 0, HARD_LEFT = 1, SOFT_LEFT = 2, SOFT_RIGHT = 3, HARD_RIGHT = 4};
enum touching_wall {LEFT_WALL = 0, RIGHT_WALL = 1, NO_WALL = -1};
enum event_flags {BUTTON_DOWN = 1, BUTTON_UP = 2, BOTH_PRESSED_BTN1 = 4}; // need to be 1 and 2 since they are bits 0001, 0010
enum button_event_flags {BOTH_PRESSED = 1, ONE_PRESSED = 2};

// LCD Task Stuff
static GLIB_Context_t glibContext;
static volatile uint32_t msTicks = 0;
#define CM_TO_PX 1/781
#define M_TO_CM 100
#define CM_TO_MM 10
#define S_TO_MS 1000
#define MS_TO_S 1/1000
#define MM_TO_M 1/1000

// Canyon Walls Stuff
#define LEFT_WALL_X (128 - CANYON_SIZE*CM_TO_PX)/2
#define RIGHT_WALL_X (128 - CANYON_SIZE*CM_TO_PX)/2 + CANYON_SIZE*CM_TO_PX

// DEBUG ----------------------------------------------------------------------
char print_string[2];
int first_interrupt = 0;

// -------------------------------------------------------------------------- //

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/
// ----------------- FINAL PROJECT SPECIFIC --------------------------------- //
static void laser_update_task(void *arg);
static void shield_enhance_task(void *arg);
static void shield_force_task(void *arg);
static void physics_update_task(void *arg);
static void led0_update_task(void *arg);
static void led1_update_task(void *arg);
static void lcd_output_task(void *arg);
void laser_valid(int num_lasers, int * data);
void led1_periodic_callback(void * p_tmr, void * p_arg);
void led1_oneshot_callback(void * p_tmr, void * p_arg);
void led0_periodic_callback(void * p_tmr, void * p_arg);
void led0_oneshot_callback(void * p_tmr, void * p_arg);
void led3_periodic_callback(void * p_tmr, void * p_arg);
void led3_oneshot_callback(void * p_tmr, void * p_arg);
int valid_config_data(void);

int read_capsense();
void draw_holtzmann(int xPos, int yPos, GLIB_Context_t * glibContext);
void draw_shield(int xPos, int yPos, GLIB_Context_t * glibContext);
void draw_canyon(GLIB_Context_t * glibContext);
void draw_town(GLIB_Context_t * glibContext);
// -------------------------------------------------------------------------- //

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  RTOS_ERR err;
//  read_button0();
  // 0 if pressed, 1 not pressed GPIO_PinInGet
  uint32_t btn0_state = GPIO_PinInGet(BUTTON0_port, BUTTON0_pin);
  uint32_t btn1_state = GPIO_PinInGet(BUTTON1_port, BUTTON1_pin);
  if(!btn0_state && !btn1_state) {
      // both buttons pressed
      OSFlagPost(&btn0_flag_group,
                 BOTH_PRESSED,
                 OS_OPT_POST_FLAG_SET,
                 &err);
  }
  else {
  // Post BTN0 Flag
    OSFlagPost(&btn0_flag_group,
               ONE_PRESSED,
               OS_OPT_POST_FLAG_SET,
               &err);
  }

  // clear interrupt flag?
  uint32_t int_flag = GPIO->IF & GPIO->IEN;
  GPIO->IFC = int_flag;
}

/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
//  read_button1();
  RTOS_ERR err;


  // 0 if pressed, 1 not pressed GPIO_PinInGet
  uint32_t btn1_state = GPIO_PinInGet(BUTTON1_port, BUTTON1_pin);
  uint32_t btn0_state = GPIO_PinInGet(BUTTON0_port, BUTTON0_pin);

  // Post BTN1 Event Flag (depending on button up or button down)
  if(!btn1_state && !btn0_state) {
      // both pressed
      OSFlagPost (&btn1_flag_group,
                  BOTH_PRESSED_BTN1,
                  OS_OPT_POST_FLAG_SET,
                  &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }
  else if(!btn1_state) {
    OSFlagPost (&btn1_flag_group,
                BUTTON_DOWN,
                OS_OPT_POST_FLAG_SET,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }
  else if(btn1_state && first_interrupt != 0){
    OSFlagPost (&btn1_flag_group,
                BUTTON_UP,
                OS_OPT_POST_FLAG_SET,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }

  first_interrupt = 1;
  // clear interrupt flag?uint32_t int_flag;
  uint32_t int_flag = GPIO->IF & GPIO->IEN;
  GPIO->IFC = int_flag;
}

/***************************************************************************//**
 * @brief
 *   Keeps current time variable.
 ******************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;
}

/***************************************************************************//**
 * Initialize blink example.
 ******************************************************************************/
void blink_init(void)
{
  RTOS_ERR err;

  // Necessary lines for SysTick Interrupt to Work
  msTicks = CMU_ClockFreqGet(cmuClock_CORE) / 1000;
  SysTick_Config(msTicks);

  // ----------------- LCD Initilization ------------------------------------ //
  uint32_t status;

  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Narrow font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);

  // Test Config Data Values
  EFM_ASSERT(valid_config_data() == 1);

  // ------------------ CREATE FINAL PROJECT DATA STRUCTURES ---------------- //
  // BTN0 Flag Group
  OSFlagCreate(&btn0_flag_group,
              "BTN0 Both Pressed Flags",
              0,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // BTN1 Flag Group
  OSFlagCreate(&btn1_flag_group,
               "BTN1 BothPressed/Pressed/Unpressed Flags",
               0,
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // ShieldData Mutex
  OSMutexCreate(&shield_mutex,
                "Shield Data Mutex",
                &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // HoltzmannData Mutex
  OSMutexCreate(&holtzmann_mutex,
                "Holtzmann Data Mutex",
                &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // GameData Mutex
  OSMutexCreate(&game_mutex,
                "Game Data Mutex",
                &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // LED0 Flag Group
  OSFlagCreate(&led0_flag_group,
               "LED0 Update Flags",
               0,
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // LED1 Flag Group
  OSFlagCreate(&led1_flag_group,
               "LED1 Update Flags",
               0,
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  // TIMERS FOR LED PWM CONTROL
  OSTmrCreate(&led1_period_timer,
              "Led1 Periodic Timer",
              0,
              LED1_INITIAL_PERIOD,
              OS_OPT_TMR_PERIODIC,
              &led1_periodic_callback,
              DEF_NULL,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSTmrCreate(&led1_oneshot_timer,
              "Led1 Oneshot Timer",
              LED1_INITIAL_ONESHOT,
              0,
              OS_OPT_TMR_ONE_SHOT,
              &led1_oneshot_callback,
              DEF_NULL,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSTmrCreate(&led0_period_timer,
              "Led0 Periodic Timer",
              0,
              LED0_INITIAL_PERIOD,
              OS_OPT_TMR_PERIODIC,
              &led0_periodic_callback,
              DEF_NULL,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSTmrCreate(&led0_oneshot_timer,
              "Led0 Oneshot Timer",
              LED0_INITIAL_ONESHOT,
              0,
              OS_OPT_TMR_ONE_SHOT,
              &led0_oneshot_callback,
              DEF_NULL,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSTmrCreate(&led3_period_timer,
              "Led3 Periodic Timer",
              0,
              LED3_INITIAL_PERIOD,
              OS_OPT_TMR_PERIODIC,
              &led3_periodic_callback,
              DEF_NULL,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSTmrCreate(&led3_oneshot_timer,
              "Led3 Oneshot Timer",
              LED3_INITIAL_ONESHOT,
              0,
              OS_OPT_TMR_ONE_SHOT,
              &led3_oneshot_callback,
              DEF_NULL,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


  // ------------------ CREATE FINAL PROJECT TASKS -------------------------- //
  // Laser Update Task
  OSTaskCreate(&laser_tcb,
               "laser update task",
               laser_update_task,
               DEF_NULL,
               LASER_TASK_PRIO,
               &laser_stack[0],
               (LASER_TASK_STACK_SIZE / 10u),
               LASER_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // Shield Enhance Task
  OSTaskCreate(&shield_enhance_tcb,
               "shield enhance task",
               shield_enhance_task,
               DEF_NULL,
               SHIELD_ENHANCE_TASK_PRIO,
               &shield_enhance_stack[0],
               (SHIELD_ENHANCE_TASK_STACK_SIZE / 10u),
               SHIELD_ENHANCE_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // Shield Force Task
  OSTaskCreate(&shield_force_tcb,
               "shield force task",
               shield_force_task,
               DEF_NULL,
               SHIELD_FORCE_TASK_PRIO,
               &shield_force_stack[0],
               (SHIELD_FORCE_TASK_STACK_SIZE / 10u),
               SHIELD_FORCE_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // Physics Update Task
  OSTaskCreate(&physics_tcb,
               "physics update task",
               physics_update_task,
               DEF_NULL,
               PHYSICS_TASK_PRIO,
               &physics_stack[0],
               (PHYSICS_TASK_STACK_SIZE / 10u),
               PHYSICS_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // LED0 Update Task
  OSTaskCreate(&led0_tcb,
               "led0 update task",
               led0_update_task,
               DEF_NULL,
               LED0_TASK_PRIO,
               &led0_stack[0],
               (LED0_TASK_STACK_SIZE / 10u),
               LED0_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // LED1 Update Task
  OSTaskCreate(&led1_tcb,
               "led1 update task",
               led1_update_task,
               DEF_NULL,
               LED1_TASK_PRIO,
               &led1_stack[0],
               (LED1_TASK_STACK_SIZE / 10u),
               LED1_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  // LCD Display Task
  OSTaskCreate(&lcd_tcb,
                 "lcd output task",
                 lcd_output_task,
                 DEF_NULL,
                 LCD_TASK_PRIO,
                 &lcd_stack[0],
                 (LCD_TASK_STACK_SIZE / 10u),
                 LCD_TASK_STACK_SIZE,
                 0u,
                 0u,
                 DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * Laser Update Task.
 ******************************************************************************/
static void laser_update_task(void *arg) {
  RTOS_ERR err;
  OS_FLAGS flags_set;
  PP_UNUSED_PARAM(arg);

  while(1) {
      // call OS function to pend on BTN0 flag group
      flags_set = OSFlagPend(&btn0_flag_group,
                             BOTH_PRESSED | ONE_PRESSED,
                             0,
                             OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME + OS_OPT_PEND_BLOCKING,
                             0,
                             &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // once here we know button0 has just been pressed
      // can now get a mutex on the game data to update the lasers
      // pend on game data mutex
      OSMutexPend(&game_mutex,
                  0,
                  OS_OPT_PEND_BLOCKING,
                  0,
                  &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      OSMutexPend(&shield_mutex,
                  0,
                  OS_OPT_PEND_BLOCKING,
                  0,
                  &err);

      if(flags_set & ONE_PRESSED) {
        int laser_active[2];
        laser_valid(game.num_laser_left, laser_active);
        game.num_laser_left = laser_active[1];
        game.laser_active = laser_active[0];
      }
      else {
          // means Both pressed and shield is destroyed
          shield.yPos = -100000;
      }

      OSMutexPost (&shield_mutex,
                   OS_OPT_POST_NONE,
                   &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // post on game data mutex
      OSMutexPost (&game_mutex,
                   OS_OPT_POST_NONE,
                   &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }
}

void laser_valid(int num_lasers, int * data) {
  data[0] = 0;
  data[1] = num_lasers;
  if(num_lasers > 0) {
      data[0] = 1;
      data[1]--;
  }
}

/***************************************************************************//**
 * Shield Enhance Task.
 ******************************************************************************/
static void shield_enhance_task(void *arg) {
  RTOS_ERR err;
  OS_FLAGS flags_set;
  PP_UNUSED_PARAM(arg);

  while(1) {
      // call OS function to pend on BTN1 semaphore
      flags_set = OSFlagPend(&btn1_flag_group,
                             BOTH_PRESSED_BTN1 | BUTTON_UP | BUTTON_DOWN,
                             0,
                             OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME + OS_OPT_PEND_BLOCKING,
                             0,
                             &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // once here, we know button1 has just been pressed
      // can now get a mutex on the shield data to set shield to enhanced and set time shield was enhanced
      // pend on shield data mutex
      OSMutexPend(&shield_mutex,
                  0,
                  OS_OPT_PEND_BLOCKING,
                  0,
                  &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // update shield enhance-data -> shield only enhanced after button interrupt IF
        // if you pulse the field by hitting the left button within a short interval before impact on the platform
        // and holding it long enough for the bounce moment
        //  the enhanced shield can only stay active for a short time
        //  before it must be inactive for period of time (which it automatically enters when the button is released)

      // shield enhanced if button pressed down AND msTicks - shield.timeUnenhanced >= SHIELD_RECHARGE_TIME
      if(flags_set & BOTH_PRESSED_BTN1) {
          // shield is destroyed if both were pressed
          shield.yPos = -100000;
      }
      if((flags_set & BUTTON_DOWN) && (msTicks - shield.timeUnenhanced >= SHIELD_RECHARGE_TIME)) {
        shield.enhanced = 1;
        shield.timeEnhanced = msTicks;
        shield.timeUnenhanced = 0;
      }
        // shield.timeUnenhanced = 0;
      // shield no longer enhanced if button unpressed && shield.enhanced = 1
      if((flags_set & BUTTON_UP) && shield.enhanced) {
        shield.enhanced = 0;
        shield.timeEnhanced = 0;
        shield.timeUnenhanced = msTicks;
      }

      // post on shield data mutex
      OSMutexPost (&shield_mutex,
                   OS_OPT_POST_NONE,
                   &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }

}

/***************************************************************************//**
 * Shield Force Task.
 ******************************************************************************/
static void shield_force_task(void *arg) {

  // initialize hardware "owned" by this task... the capsense gpio?
  CAPSENSE_Init();

  RTOS_ERR err;

  int force_applied = 0;

  PP_UNUSED_PARAM(arg);

  // infinite while loop
  while (1)
  {
      // call OSTimeDly to delay between each loop iteration
      OSTimeDly(10, OS_OPT_TIME_DLY, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // add call to read capacitive touch slider
      int curr_dir = read_capsense();

      if(curr_dir == HARD_LEFT) {
        force_applied = -SHIELD_MAX_FORCE;
      }
      else if(curr_dir == SOFT_LEFT) {
          force_applied = -SHIELD_MAX_FORCE/2;
      }
      else if(curr_dir == SOFT_RIGHT) {
          force_applied = SHIELD_MAX_FORCE/2;
      }
      else if(curr_dir == HARD_RIGHT) {
          force_applied = SHIELD_MAX_FORCE;
      }
      else {
          force_applied = 0;
      }

      // pend on shield data mutex
      OSMutexPend(&shield_mutex,
                  0,
                  OS_OPT_PEND_BLOCKING,
                  0,
                  &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // update shield data
      shield.forceApplied = force_applied;

      // post on shield data mutex
      OSMutexPost (&shield_mutex,
                   OS_OPT_POST_NONE,
                   &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }

}

/***************************************************************************//**
 * Physics Update Task.
 ******************************************************************************/
static void physics_update_task(void *arg) {
  PP_UNUSED_PARAM(arg);
  RTOS_ERR err;
  // update the position, velocity, and acceleration of the Shield Platform
    // if ax = 0, velocity unchanged, else velocity changes
    // if vx = 0, position unchanged, else position changes
    // if finger on slider, constant acceleration applied
    // if platform collides with wall,
  // update the position, velocity, and acceleration of the Holtzmann Mass

  // infinite while loop
  while (1)
  {
    // call OSTimeDly to delay between each loop iteration
    OSTimeDly(TAU_PHYSICS, OS_OPT_TIME_DLY, &err);

    // pend on game data mutex
    OSMutexPend(&game_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // pend on shield data mutex
    OSMutexPend(&shield_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // pend on holtzmann data mutex
    OSMutexPend(&holtzmann_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // Check if shield touching a wall
    int shield_touching_wall = NO_WALL;
    if((shield.xPos - 0.5*SHIELD_LENGTH) <= -0.5*CANYON_SIZE) {
        shield_touching_wall = LEFT_WALL;
    }
    if((shield.xPos + 0.5*SHIELD_LENGTH) >= 0.5*CANYON_SIZE) {
      shield_touching_wall = RIGHT_WALL;
    }
    // Update shield x physics
    if(shield_touching_wall == NO_WALL) {
      shield.xAccl = (shield.forceApplied/HM_MASS)*M_TO_CM; // (N / kg)*(100 cm/m) -> (m/s^2)*(100 cm/m) -> cm/s^2
      shield.xVel = shield.xVel + shield.xAccl*TAU_PHYSICS*MS_TO_S; // cm/s + (cm/s^2)*(ms)*(1 s / 1000 ms) -> cm/s
      shield.xPos = shield.xPos + shield.xVel*TAU_PHYSICS*MS_TO_S; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm
    }
    else {
        shield.xAccl = 0;
        shield.xVel = -shield.xVel;
        shield.xPos = shield.xPos + shield.xVel*TAU_PHYSICS*MS_TO_S; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm
    }

    // Check if HM touching wall
    int HM_touching_wall = NO_WALL;
    // if left side of ball (xPos - half diameter) is off the left side of the canyon or touching the wall
    if((holtzmann.xPos - 0.5*HM_DIAM) <= -0.5*CANYON_SIZE) {
        HM_touching_wall = LEFT_WALL;
    }
    if((holtzmann.xPos + 0.5*HM_DIAM) >= 0.5*CANYON_SIZE) {
        HM_touching_wall = RIGHT_WALL;
    }

    // Check if HM touching platform
    int HM_touching_platform = 0;
    // if bottom of ball is below the platform or touching it AND middle of ball is within bounds of platform x
    if(((holtzmann.yPos - 0.5*HM_DIAM) <= (shield.yPos + 0.5*SHIELD_HEIGHT)) &&
       ((holtzmann.yPos - 0.5*HM_DIAM) > (shield.yPos - 0.5*SHIELD_HEIGHT)) &&
        (holtzmann.xPos <= shield.xPos + 0.5*SHIELD_LENGTH) &&
        (holtzmann.xPos >= shield.xPos - 0.5*SHIELD_LENGTH)) {
        HM_touching_platform = 1;
    }
    // if HM touching platform, check if shield enhanced
    if(HM_touching_platform) {
      // (if shield.enhanced == 1, need to check if still within time range it works)
      if(shield.enhanced) {
          if(msTicks - shield.timeEnhanced > SHIELD_ARMING_WINDOW) {
              // if not within time range means button got pressed too early before impact
              EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
              shield.enhanced = 0;
              shield.timeEnhanced = 0;
              shield.timeUnenhanced = msTicks;
              EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
          }
      }
      // if not, means button got released and isn't currently held down, no changes to shield.enhanced needed
    }

    // Update HM mass y physics: 4 Possible Cases:
    if(HM_touching_platform && abs(holtzmann.yVel) < HM_MIN_SPEED) {
        // Touching Platform (Too Slow)
        holtzmann.yForce = GRAVITY*HM_MASS*MM_TO_M; // (mm/s^2)*kg*(1 m / 1000 mm) -> N (kg*m/s^2)
        holtzmann.yAccl = holtzmann.yForce/HM_MASS*M_TO_CM; // (N / kg)*(100 cm/m) -> (m/s^2)*(100 cm/m) -> cm/s^2
        holtzmann.yVel = holtzmann.yVel + holtzmann.yAccl*TAU_PHYSICS*MS_TO_S; // cm/s + (cm/s^2)*(ms)*(1 s / 1000 ms) -> cm/s
        holtzmann.yPos = shield.yPos - 0.5*SHIELD_HEIGHT; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm
    }
    else if(HM_touching_platform && shield.enhanced) {
        // Touching Platform Enhanced -> increase KE by SHIELD_ENERGY_INCREASE % (KE = 1/2*mass*vel^2
        long double_pre_energy_div_by_mass = (holtzmann.yVel)*(holtzmann.yVel); // kg*(cm/s)^2
        long double_post_energy_div_by_mass = double_pre_energy_div_by_mass + (float)((float)SHIELD_ENERGY_INCREASE/(float)100)*(float)double_pre_energy_div_by_mass; // kg*(cm/s)^2
        int post_vel = sqrt(double_post_energy_div_by_mass);

        holtzmann.yAccl = 0; // cm/s^2
        holtzmann.yVel = post_vel; //(int)sqrt(2*post_energy/HM_MASS); // vel = sqrt(2*KE/mass) = cm/s
        holtzmann.yPos = holtzmann.yPos + 0.5*SHIELD_HEIGHT + 1; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm
    }
    else if(HM_touching_platform && !shield.enhanced) {
        // Touching Platform Unenhanced -> decrease KE by SHIELD_ENERGY_REDUCTION
        long double_pre_energy_div_by_mass = (holtzmann.yVel)*(holtzmann.yVel); // kg*(cm/s)^2
        long double_post_energy_div_by_mass = double_pre_energy_div_by_mass - (float)((float)SHIELD_ENERGY_REDUCTION/(float)100)*(float)double_pre_energy_div_by_mass; // kg*(cm/s)^2
        int post_vel = sqrt(double_post_energy_div_by_mass);

        holtzmann.yAccl = 0; // cm/s^2
        holtzmann.yVel = post_vel; //(int)sqrt(2*post_energy/HM_MASS); // vel = sqrt(2*KE/mass) = cm/s
        holtzmann.yPos = holtzmann.yPos + 0.5*SHIELD_HEIGHT + 1; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm
    }
    else {
        // Free Fall
        holtzmann.yForce = GRAVITY*HM_MASS*MM_TO_M; // (mm/s^2)*kg*(1 m / 1000 mm) -> N (kg*m/s^2)
        holtzmann.yAccl = holtzmann.yForce/HM_MASS*M_TO_CM; // (N / kg)*(100 cm/m) -> (m/s^2)*(100 cm/m) -> cm/s^2
        holtzmann.yVel = holtzmann.yVel + holtzmann.yAccl*TAU_PHYSICS*MS_TO_S; // cm/s + (cm/s^2)*(ms)*(1 s / 1000 ms) -> cm/s
        holtzmann.yPos = holtzmann.yPos + holtzmann.yVel*TAU_PHYSICS*MS_TO_S; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm
    }

    // Update HM mass x physics: 3 Possible Cases:
    if(HM_touching_wall == LEFT_WALL) {
        // Left Wall Hit
        holtzmann.xForce = 0;
        holtzmann.xAccl = 0;
        holtzmann.xVel = -holtzmann.xVel; // cm/s + (cm/s^2)*(ms)*(1 s / 1000 ms) -> cm/s
        holtzmann.xPos = holtzmann.xPos + holtzmann.xVel*TAU_PHYSICS*MS_TO_S; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm

    }
    else if(HM_touching_wall == RIGHT_WALL) {
        // Right Wall Hit
        holtzmann.xForce = 0;
        holtzmann.xAccl = 0;
        holtzmann.xVel = -holtzmann.xVel; // cm/s + (cm/s^2)*(ms)*(1 s / 1000 ms) -> cm/s
        holtzmann.xPos = holtzmann.xPos + holtzmann.xVel*TAU_PHYSICS*MS_TO_S; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm

    }
    else {
        // Free Fall
        holtzmann.xForce = 0;
        holtzmann.xAccl = 0;
        holtzmann.xVel = holtzmann.xVel + holtzmann.xAccl*TAU_PHYSICS*MS_TO_S; // cm/s + (cm/s^2)*(ms)*(1 s / 1000 ms) -> cm/s
        holtzmann.xPos = holtzmann.xPos + holtzmann.xVel*TAU_PHYSICS*MS_TO_S; // cm + (cm/s)*(ms)*(1 s / 1000 ms) -> cm
    }

    // if HM mass goes off top of screen, reset it and decrement num_HM_left
    if(holtzmann.yPos - 0.5*HM_DIAM >= 100000) {
        holtzmann.yPos = 100000; // reset HM position
        holtzmann.yVel = HM_INITIAL_Y_VELOCITY; // and velocity
        game.num_HM_left--;
    }

    // update game data
    if(game.laser_active) {
        game.num_HM_left--; // one HM destroyed
        holtzmann.yPos = 100000; // reset HM position
        holtzmann.yVel = HM_INITIAL_Y_VELOCITY; // and velocity
        game.laser_active = 0;
    }

    // game over -> lose
    // if hm goes off bottom of screen
    if(holtzmann.yPos + 0.5*HM_DIAM < 0) {
        game.game_over = -1;
    }

    // game over -> win
    // if num_HM_left becomes 0
    if(game.num_HM_left == 0) {
       game.game_over = 1;
    }

    // post on holtzmann data mutex
    OSMutexPost (&holtzmann_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // post on shield data mutex
    OSMutexPost (&shield_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // post on game data mutex
    OSMutexPost (&game_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  }
}

/***************************************************************************//**
 * LED0 Update Task.
 ******************************************************************************/
static void led0_update_task(void *arg) {
  RTOS_ERR err;

  PP_UNUSED_PARAM(arg);

  // start led0 periodic timer
  OSTmrStart(&led0_period_timer,
             &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


  // infinite while loop
  while (1)
  {
    // call OSTimeDly to delay between each loop iteration
    OSTimeDly(10, OS_OPT_TIME_DLY, &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // Normally this is used to provide simple guidance to your platform operator.
    // Assuming that the HM???s current trajectory and the platform???s trajectory would
    // not hit a canyon wall, pulse width modulate this LED to show the % of MAX_FORCE
    // that would need to be applied to the platform to get it to intersect the HM center-on.

    // If an HM has gotten through the shield or onto the ground thereby leaving the base open

    // Need to get game data mutex first, if game over -> 50% duty cycle, blink at 1 Hz
    int game_over = 0;
    // pend on game data mutex
    OSMutexPend(&game_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    game_over = game.game_over;

    // post on game data mutex
    OSMutexPost (&game_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // if game not over -> get holtzmann and shield data mutexes and calculate stuff
    int duty_cycle_percent = 0;
    if(game_over != -1) {
        // pend on shield data mutex
        OSMutexPend(&shield_mutex,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    0,
                    &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // pend on holtzmann data mutex
        OSMutexPend(&holtzmann_mutex,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    0,
                    &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // calculate % of Max Force needed to line up HM with shield
        // if difference in their x values (keep in mind origin in center) is certain amount -> max force
        if(abs(holtzmann.xPos - shield.xPos) > 50000) {
            duty_cycle_percent = 100;
        }
        // if difference in x values is smaller than that -> less force (half?)
        if(abs(holtzmann.xPos - shield.xPos) <= 50000 && abs(holtzmann.xPos - shield.xPos) > SHIELD_LENGTH/2) {
            duty_cycle_percent = 50;
        }
        // if difference in x values is 0
        if(abs(holtzmann.xPos - shield.xPos) <= SHIELD_LENGTH/2) {
            duty_cycle_percent = 0;
        }

        // post on holtzmann data mutex
        OSMutexPost (&holtzmann_mutex,
                     OS_OPT_POST_NONE,
                     &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // post on shield data mutex
        OSMutexPost (&shield_mutex,
                     OS_OPT_POST_NONE,
                     &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // set period of led0_oneshot_timer to make display at according duty cycle %
        int one_shot_delay = LED0_INITIAL_ONESHOT;
        if(duty_cycle_percent == 100) {
            one_shot_delay = LED0_INITIAL_PERIOD;
        }
        else if(duty_cycle_percent == 50) {
            one_shot_delay = LED0_INITIAL_ONESHOT; // 10 ms
        }

        OSTmrSet(&led0_oneshot_timer,
                 one_shot_delay,
                 0,
                 &led0_oneshot_callback,
                 &duty_cycle_percent,
                 &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        OSTmrSet(&led0_period_timer,
                 0,
                 LED0_INITIAL_PERIOD,
                 &led0_periodic_callback,
                 &duty_cycle_percent,
                 &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
    }
    else {
        // blink on/off with a 50% duty cycle at 1Hz to alert all nearby personnel to evacuate.

        // turn off other led0 timer
        OSTmrStop(&led0_period_timer,
                  OS_OPT_TMR_NONE,
                  DEF_NULL,
                  &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        duty_cycle_percent = 50;
        // set oneshot timer to 50% duty cycle
        OSTmrSet(&led0_oneshot_timer,
                 LED0_INITIAL_ONESHOT,
                 0,
                 &led0_oneshot_callback,
                 &duty_cycle_percent,
                 &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        OSTmrSet(&led0_period_timer,
                 0,
                 LED0_INITIAL_PERIOD,
                 &led0_periodic_callback,
                 &duty_cycle_percent,
                 &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        // may need another timer at 1 Hz frequency, so during the on second other 2 timers run, during off other timers stop and led off
        // start 1 Hz timer that turns led on / off
        OSTmrStart(&led3_period_timer,
                   &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // then break out of loop, this task is finished
        break;
    }
  }
}

/***************************************************************************//**
 * LED1 Update Task.
 ******************************************************************************/
static void led1_update_task(void *arg) {
  RTOS_ERR err;

  PP_UNUSED_PARAM(arg);

  // start led1 periodic timer
  OSTmrStart(&led1_period_timer,
             &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  // infinite while loop
  while (1)
  {
      // call OSTimeDly to delay between each loop iteration
      OSTimeDly(10, OS_OPT_TIME_DLY, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      // Pulse width modulated to show (via human-perceived brightness) the current force magnitude, as a % of MAX_FORCE.
      // need to calculate duty_cycle_percentage = current_force_mag / MAX_FORCE;
      // need mutex on shield data
      int current_force_mag = 0;
      OSMutexPend(&shield_mutex,
                  0,
                  OS_OPT_PEND_BLOCKING,
                  0,
                  &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      current_force_mag = abs(shield.forceApplied);

      // post on shield data mutex
      OSMutexPost (&shield_mutex,
                   OS_OPT_POST_NONE,
                   &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      int one_shot_delay = LED1_INITIAL_ONESHOT;
      int led1_callback_arg = 0;
      int led1_periodic_arg = 0;
      if(current_force_mag == SHIELD_MAX_FORCE) {
          one_shot_delay = LED1_INITIAL_PERIOD; // ms -> 100% duty cycle
          led1_periodic_arg = 1;
      }
      else if(current_force_mag == SHIELD_MAX_FORCE/2) {
          one_shot_delay = LED1_INITIAL_ONESHOT; // ms -> 50% duty cycle
      }
      else {
          led1_callback_arg = 1; // 0% duty cycle
          // turn off light and stop periodic
      }

      OSTmrSet(&led1_oneshot_timer,
               one_shot_delay,
               0,
               &led1_oneshot_callback,
               &led1_callback_arg,
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      OSTmrSet(&led1_period_timer,
               0,
               LED1_INITIAL_PERIOD,
               &led1_periodic_callback,
               &led1_periodic_arg,
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }
}

/***************************************************************************//**
 * LCD Task.
 ******************************************************************************/
static void lcd_output_task(void *arg)
{
    RTOS_ERR err;

    PP_UNUSED_PARAM(arg);

    // infinite while loop
    while (1)
    {
        // call OSTimeDly to delay between each loop iteration
        OSTimeDly(TAU_LCD, OS_OPT_TIME_DLY, &err);

        // reset the screen
        GLIB_clear(&glibContext);

        int shield_xPos = SHIELD_INITIAL_X;
        int shield_yPos = SHIELD_INITIAL_Y;

        int holtzmann_xPos = HM_INITIAL_X_POSITION;
        int holtzmann_yPos = HM_INITIAL_Y_POSITION;

        int game_over = 0;
        int num_hm = 0;
        int num_laser = 0;

        // pend on game data mutex
        OSMutexPend (&game_mutex,
                     0,
                     OS_OPT_PEND_BLOCKING,
                     0,
                     &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // pend on shield data mutex
        OSMutexPend(&shield_mutex,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    0,
                    &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // pend on holtzmann data mutex
        OSMutexPend(&holtzmann_mutex,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    0,
                    &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        game_over = game.game_over;
        num_hm = game.num_HM_left;
        num_laser = game.num_laser_left;

        // get shield data
        shield_xPos = shield.xPos;
        shield_yPos = shield.yPos;

        holtzmann_xPos = holtzmann.xPos;
        holtzmann_yPos = holtzmann.yPos;

        // post on holtzmann data mutex
        OSMutexPost (&holtzmann_mutex,
                     OS_OPT_POST_NONE,
                     &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // post on shield data mutex
        OSMutexPost (&shield_mutex,
                     OS_OPT_POST_NONE,
                     &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // post on game data mutex
        OSMutexPost (&game_mutex,
                     OS_OPT_POST_NONE,
                     &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // create strings for num HM and num lasers
        char num_hm_string[7];
        char num_laser_string[12];

        sprintf(num_hm_string, "HMs: %d", num_hm);
        sprintf(num_laser_string, "Lasers: %d", num_laser);

        if(game_over == 0) {
  //        draw_town(&glibContext);
          draw_holtzmann(holtzmann_xPos, holtzmann_yPos, &glibContext);
          draw_shield(shield_xPos, shield_yPos, &glibContext);
          draw_canyon(&glibContext);

          GLIB_drawString (&glibContext,
                           num_hm_string,
                           strlen(num_hm_string),
                           5,
                           5,
                           true
          );
          GLIB_drawString (&glibContext,
                           num_laser_string,
                           strlen(num_laser_string),
                           5,
                           15,
                           true
          );
        }
        else if(game_over == -1) {
            GLIB_drawString (&glibContext,
                             "YOU LOSE",
                             strlen("YOU LOSE"),
                             40,
                             60,
                             true
            );
        }
        else if(game_over == 1) {
            GLIB_drawString (&glibContext,
                             "YOU WIN",
                             strlen("YOU WIN"),
                             45,
                             60,
                             true
            );
        }

        DMD_updateDisplay();
    }
}

/***************************************************************************//**
 * @brief
 *   Sample pushbutton 1 only if pushbutton 0 is not currently pressed.
 ******************************************************************************/
int read_capsense() {
  //getPressed (true or false 0-1-2-3) or getSliderPosition (0 - 255)
  CAPSENSE_Sense();
  //uint32_t slide_position = CAPSENSE_getSliderPosition();
  bool far_left = CAPSENSE_getPressed(0);
  bool left = CAPSENSE_getPressed(1);
  bool right = CAPSENSE_getPressed(2);
  bool far_right = CAPSENSE_getPressed(3);

  int slider_dir;

  if(far_left && !right && !far_right) {
    slider_dir = HARD_LEFT;
    print_string[0] = 'H';
    print_string[1] = 'L';
  }
  else if (left && !right && !far_right) {
    slider_dir = SOFT_LEFT;
    print_string[0] = 'S';
    print_string[1] = 'L';
  }
  else if (!far_left && !left && right) {
    slider_dir = SOFT_RIGHT;
    print_string[0] = 'S';
    print_string[1] = 'R';
  }
  else if (!far_left && !left && far_right) {
    slider_dir = HARD_RIGHT;
    print_string[0] = 'H';
    print_string[1] = 'R';
  }
  else {
    slider_dir = STRAIGHT;
    print_string[0] = 'S';
    print_string[1] = 'T';
  }

  return slider_dir;
}

/***************************************************************************//**
 * @brief
 *   Draw holtzmann mass on LCD screen at given xPos and yPos.
 ******************************************************************************/
void draw_holtzmann(int xPos, int yPos, GLIB_Context_t * glibContext) {
  // need to translate xPos and yPos from cm to px
  // xPos range: [-0.5*CANYON_SIZE, 0,5*CANYON_SIZE]cm -> [0, 128]px x_px = 0.00128*x_Pos + 64
  // yPos range: [100000, 0]cm -> [0, 128]px y_px = -0.00128*yPos + 128
  int x_px = 0.00128*xPos + 64;
  int y_px = -0.00128*yPos + 128;

  if(HM_IMAGE == 0) {
    GLIB_drawCircleFilled(glibContext,
                          x_px,
                          y_px,
                          HM_DIAM*CM_TO_PX/2);
  }
  else if(HM_IMAGE == 1) {
      GLIB_drawBitmap(glibContext, xPos-0.5*PERSON2_WIDTH, yPos-0.5*PERSON2_HEIGHT, PERSON2_WIDTH, PERSON2_HEIGHT, person2_bits);
  }
}

/***************************************************************************//**
 * @brief
 *   Draw shield platform on LCD screen at given xPos and yPos.
 ******************************************************************************/
void draw_shield(int xPos, int yPos, GLIB_Context_t * glibContext) {
  // need to translate xPos and yPos from cm to px
  // xPos range: [-0.5*CANYON_SIZE, 0,5*CANYON_SIZE]cm -> [0, 128]px x_px = 0.00128*x_Pos + 64
  // yPos range: [100000, 0]cm -> [0, 128]px y_px = -0.00128*yPos + 128
  int x_px = 0.00128*xPos + 64;
  int y_px = -0.00128*yPos + 128;

  GLIB_Rectangle_t shield_platform;
  shield_platform.xMax = x_px + SHIELD_LENGTH*CM_TO_PX/2;
  shield_platform.xMin = x_px - SHIELD_LENGTH*CM_TO_PX/2;
  shield_platform.yMax = y_px + SHIELD_HEIGHT*CM_TO_PX/2;
  shield_platform.yMin = y_px - SHIELD_HEIGHT*CM_TO_PX/2;

  GLIB_drawRectFilled(glibContext,
                      &shield_platform);
}

/***************************************************************************//**
 * @brief
 *   Draw canyon on LCD screen of size CANYON_SIZE
 ******************************************************************************/
void draw_canyon(GLIB_Context_t * glibContext) {
  GLIB_Rectangle_t left_wall;
  GLIB_Rectangle_t right_wall;

  left_wall.xMax = LEFT_WALL_X;
  left_wall.xMin = 0;
  left_wall.yMax = 128;
  left_wall.yMin = 0;

  right_wall.xMax = 128;
  right_wall.xMin = RIGHT_WALL_X;
  right_wall.yMax = 128;
  right_wall.yMin = 0;

  GLIB_drawRectFilled(glibContext,
                      &left_wall);

  GLIB_drawRectFilled(glibContext,
                      &right_wall);
}

/***************************************************************************//**
 * @brief
 *   Draw town on LCD screen at bottom of canyon.
 ******************************************************************************/
void draw_town(GLIB_Context_t * glibContext) {
  GLIB_drawBitmap(glibContext, 0, 0, TOWN_WIDTH, TOWN_HEIGHT, town_bits);
}

void led1_periodic_callback(void * p_tmr, void *p_arg) {

  PP_UNUSED_PARAM(p_tmr);

  RTOS_ERR err;

  int * max_force_applied = p_arg;

  // turn on led1
  if(*max_force_applied)
    GPIO_PinOutSet(LED1_port, LED1_pin);
  else
    GPIO_PinOutClear(LED1_port, LED1_pin);

//   start led1_oneshot_timer
  OSTmrStart(&led1_oneshot_timer,
             &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}

void led1_oneshot_callback(void * p_tmr, void *p_arg) {

  PP_UNUSED_PARAM(p_tmr);

  int * no_force_applied = p_arg;

  // turn off led1
  if(*no_force_applied)
      GPIO_PinOutClear(LED1_port, LED1_pin);
  else
      GPIO_PinOutSet(LED1_port, LED1_pin);

}

void led0_periodic_callback(void * p_tmr, void *p_arg) {

  PP_UNUSED_PARAM(p_tmr);

  RTOS_ERR err;

  int * duty_cycle_percent = p_arg;

  // turn off led0
  if(*duty_cycle_percent == 100)
    GPIO_PinOutSet(LED0_port, LED0_pin);
  else
    GPIO_PinOutClear(LED0_port, LED0_pin);

  // start led0_oneshot_timer
  OSTmrStart(&led0_oneshot_timer,
             &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

void led0_oneshot_callback(void * p_tmr, void *p_arg) {

  PP_UNUSED_PARAM(p_tmr);

  int * duty_cycle_percent = p_arg;

  // turn on led0
  if(*duty_cycle_percent == 0)
    GPIO_PinOutClear(LED0_port, LED0_pin);
  else
    GPIO_PinOutSet(LED0_port, LED0_pin);
}

void led3_periodic_callback(void * p_tmr, void *p_arg) {
  PP_UNUSED_PARAM(p_arg);
  PP_UNUSED_PARAM(p_tmr);
  RTOS_ERR err;

  // start led0 periodic timer
  GPIO_PinOutSet(LED0_port, LED0_pin);
//  OSTmrStart(&led0_period_timer,
//             &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  OSTmrStart(&led3_oneshot_timer,
             &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  // wait for half of timer period (1 Hz)
  OSTimeDly(10, OS_OPT_TIME_DLY, &err);
}

void led3_oneshot_callback(void * p_tmr, void *p_arg) {

  PP_UNUSED_PARAM(p_tmr);
  PP_UNUSED_PARAM(p_arg);

  // then turn off led0 periodic timer and led0
  GPIO_PinOutClear(LED0_port, LED0_pin);
//  OSTmrStop(&led0_period_timer,
//            OS_OPT_TMR_NONE,
//            DEF_NULL,
//            &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

int valid_config_data(void) {
  // Data Structure Version
  if(DATA_STRUCTURE_VERSION != 4)
    return 0;
  // TauPhysics
  if(TAU_PHYSICS > 100 || TAU_PHYSICS < 8)
    return 0;
  // TauLCD
  if(TAU_LCD > 150 || TAU_LCD < 10)
    return 0;
  // Gravity
  if(GRAVITY > -30000 || GRAVITY < -120000)
    return 0;
  // CanyonSize
  if(CANYON_SIZE > 100000 || CANYON_SIZE < 10000)
    return 0;

  // HM Num
  if(HM_NUM < 1)
    return 0;
  // HM Diam
  if(HM_DIAM < 1000 || HM_DIAM > 25000)
    return 0;
  // HM Init Conditions
  if(HM_INIT_CONDITIONS != 0)
    return 0;
  // HM Init xVel
  if(HM_INITIAL_X_VELOCITY < -20000 || HM_INITIAL_X_VELOCITY > 20000)
    return 0;
  // HM Init yVel
  if(HM_INITIAL_Y_VELOCITY != 0)
    return 0;
  // HM Init xPos
  if(HM_INITIAL_X_POSITION < -(CANYON_SIZE)/2 + HM_DIAM || HM_INITIAL_X_POSITION > (CANYON_SIZE)/2 - HM_DIAM)
    return 0;
  // HM user defined mode
  if(HM_USER_DEFINED_MODE_INPUT != 0)
    return 0;

  // Max Shield Force
  if(SHIELD_MAX_FORCE < 80000 || SHIELD_MAX_FORCE > 500000)
    return 0;
  // Shield Mass
  if(SHIELD_MASS < 10 || SHIELD_MASS > 10000)
    return 0;
  // Shield Length
  if(SHIELD_LENGTH < 5000 || SHIELD_LENGTH > 50000)
    return 0;
  // Bounce Enabled
  if(SHIELD_BOUNCE_ENABLED != 1)
    return 0;
  // Bounce Limited
  if(SHIELD_BOUNCE_LIMITED != 0)
    return 0;
  // Max Platform Bounce Speed
  if(SHIELD_MAX_BOUNCE_SPEED != 0)
    return 0;
  // Shield Auto Control
  if(SHIELD_AUTO_CONTROL != 0)
    return 0;
  // Shiled Min Perpendicular Speed
  if(SHIELD_MIN_PERPENDICULAR_SPEED != 1000)
    return 0;
  // Energy Reduction Percent
  if(SHIELD_ENERGY_REDUCTION > 90 || SHIELD_ENERGY_REDUCTION < 10)
    return 0;
  // Energy Increase Percent
  if(SHIELD_ENERGY_INCREASE > 70 || SHIELD_ENERGY_INCREASE < 10)
    return 0;
  // Arming Window
  if(SHIELD_ARMING_WINDOW < 500 || SHIELD_ARMING_WINDOW > 1000)
    return 0;
  // Recharge Time
  if(SHIELD_RECHARGE_TIME < 500 || SHIELD_RECHARGE_TIME > 10000)
    return 0;
  // Num Lasers
  if(NUM_LASERS < 0)
    return 0;
  // Auto Laser Control
  if(AUTO_LASERS != 0)
    return 0;

  return 1;
}
