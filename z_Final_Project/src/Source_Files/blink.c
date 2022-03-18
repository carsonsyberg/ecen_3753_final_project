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
#include "sl_board_control.h"
#include "sl_board_control_config.h"

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
static OS_TCB idle_tcb;
static CPU_STK idle_stack[IDLE_TASK_STACK_SIZE];

// Speed Setpoint Task
static OS_TCB speed_tcb;
static CPU_STK speed_stack[SPEED_TASK_STACK_SIZE];
// Vehicle Direction Task
static OS_TCB dir_tcb;
static CPU_STK dir_stack[DIR_TASK_STACK_SIZE];
// Vehicle Monitor Task
static OS_TCB monitor_tcb;
static CPU_STK monitor_stack[MONITOR_TASK_STACK_SIZE];
// LED Output Task
static OS_TCB led_tcb;
static CPU_STK led_stack[LED_TASK_STACK_SIZE];

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
                              .forceApplied = 0
                            };

struct holtzmann_data holtzmann = { .xAccl = 0,
                                    .xForce = 0,
                                    .xPos = HM_INITIAL_X_POSITION,
                                    .xVel = HM_INITIAL_X_VELOCITY,
                                    .yAccl = 0,
                                    .yForce = GRAVITY,
                                    .yPos = 0,
                                    .yVel = 0
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
// BTN0 Semaphore
static OS_SEM btn0_semaphore;
// BTN1 Semaphore
static OS_SEM btn1_semaphore;
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

enum direction_vals {STRAIGHT = 0, HARD_LEFT = 1, SOFT_LEFT = 2, SOFT_RIGHT = 3, HARD_RIGHT = 4};

// LCD Task Stuff
static GLIB_Context_t glibContext;
static volatile uint32_t msTicks = 0;

// Canyon Walls Stuff
int left_wall_x = (128 - CANYON_SIZE)/2;
int right_wall_x = (128 - CANYON_SIZE)/2 + CANYON_SIZE;

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
static void idle_task(void *arg);
static void lcd_output_task(void *arg);

int read_capsense();
void draw_holtzmann(int xPos, int yPos, GLIB_Context_t * glibContext);
void draw_shield(int xPos, int yPos, GLIB_Context_t * glibContext);
void draw_canyon(GLIB_Context_t * glibContext);
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
//  read_button0();
  RTOS_ERR err;

  // Post BTN0 Semaphore

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

  // do stuff with timer to tell if button pressed close enough to collision
  // post BTN1 Semaphore

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

  // ------------------ CREATE FINAL PROJECT DATA STRUCTURES ---------------- //
  // BTN0 Semaphore
  OSSemCreate(&btn0_semaphore,
              "BTN0 Semaphore",
              0,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // BTN1 Semaphore
  OSSemCreate(&btn1_semaphore,
              "BTN1 Semaphore",
              1,
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

  // ------------------ CREATE FINAL PROJECT TASKS -------------------------- //
  // Laser Update Task
//  OSTaskCreate(&laser_tcb,
//               "laser update task",
//               laser_update_task,
//               DEF_NULL,
//               LASER_TASK_PRIO,
//               &laser_stack[0],
//               (LASER_TASK_STACK_SIZE / 10u),
//               LASER_TASK_STACK_SIZE,
//               0u,
//               0u,
//               DEF_NULL,
//               (OS_OPT_TASK_STK_CLR),
//               &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // Shield Enhance Task
//  OSTaskCreate(&shield_enhance_tcb,
//               "shield enhance task",
//               shield_enhance_task,
//               DEF_NULL,
//               SHIELD_ENHANCE_TASK_PRIO,
//               &shield_enhance_stack[0],
//               (SHIELD_ENHANCE_TASK_STACK_SIZE / 10u),
//               SHIELD_ENHANCE_TASK_STACK_SIZE,
//               0u,
//               0u,
//               DEF_NULL,
//               (OS_OPT_TASK_STK_CLR),
//               &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
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
//  OSTaskCreate(&led0_tcb,
//               "led0 update task",
//               led0_update_task,
//               DEF_NULL,
//               LED0_TASK_PRIO,
//               &led0_stack[0],
//               (LED0_TASK_STACK_SIZE / 10u),
//               LED0_TASK_STACK_SIZE,
//               0u,
//               0u,
//               DEF_NULL,
//               (OS_OPT_TASK_STK_CLR),
//               &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//  // LED1 Update Task
//  OSTaskCreate(&led1_tcb,
//               "led1 update task",
//               led1_update_task,
//               DEF_NULL,
//               LED1_TASK_PRIO,
//               &led1_stack[0],
//               (LED1_TASK_STACK_SIZE / 10u),
//               LED1_TASK_STACK_SIZE,
//               0u,
//               0u,
//               DEF_NULL,
//               (OS_OPT_TASK_STK_CLR),
//               &err);
//  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

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

  // Create Idle Task
  OSTaskCreate(&idle_tcb,
               "idle task",
               idle_task,
               DEF_NULL,
               IDLE_TASK_PRIO,
               &idle_stack[0],
               (IDLE_TASK_STACK_SIZE / 10u),
               IDLE_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * Idle task.
 ******************************************************************************/
static void idle_task(void *arg)
{
    // initialize hardware "owned" by this task... none?

    PP_UNUSED_PARAM(arg);

    // infinite while loop
    RTOS_ERR err;
    while (1)
    {
        // call the function that performs the processing specific to this task
        EMU_EnterEM1();
        // call OSTimeDly to delay between each loop iteration
        OSTimeDly(10, OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    }


}

/***************************************************************************//**
 * Laser Update Task.
 ******************************************************************************/
static void laser_update_task(void *arg) {

}

/***************************************************************************//**
 * Shield Enhance Task.
 ******************************************************************************/
static void shield_enhance_task(void *arg) {

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
    OSTimeDly(120, OS_OPT_TIME_DLY, &err);
    // pend on shield data mutex
    OSMutexPend(&shield_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // update shield data
    shield.xAccl = shield.forceApplied/SHIELD_MASS;
    shield.xVel = shield.xVel + shield.xAccl;
    shield.xPos = shield.xPos + shield.xVel;

    // check if touching a wall -> if so reverse its velocity and make acceleration 0
    if(shield.xPos - SHIELD_LENGTH/2 < left_wall_x) {
        if(SHIELD_BOUNCE_ENABLED)
          shield.xVel = -shield.xVel - 1;
        else
          shield.xVel = 0;
        shield.xAccl = 0;
        shield.forceApplied = 0;
        shield.xPos = left_wall_x + 1 + SHIELD_LENGTH/2;
    }

    if(shield.xPos + SHIELD_LENGTH/2 > right_wall_x) {
        if(SHIELD_BOUNCE_ENABLED)
          shield.xVel = -shield.xVel + 1;
        else
          shield.xVel = 0;
        shield.xAccl = 0;
        shield.forceApplied = 0;
        shield.xPos = right_wall_x - 1 - SHIELD_LENGTH/2;
    }

    // pend on holtzmann data mutex
    OSMutexPend(&holtzmann_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // update holtzmann data
    holtzmann.xAccl = holtzmann.xForce/HM_MASS;
    holtzmann.yAccl = holtzmann.yForce/HM_MASS;
    holtzmann.xVel = holtzmann.xVel + holtzmann.xAccl;
    holtzmann.yVel = holtzmann.yVel + holtzmann.yAccl;
    holtzmann.xPos = holtzmann.xPos + holtzmann.xVel;
    holtzmann.yPos = holtzmann.yPos + holtzmann.yVel;

    // check if holtzmann mass touching platform -> if so reverse its y velocity
    if(holtzmann.yPos + HM_DIAM/2 > shield.yPos - SHIELD_HEIGHT/2 &&
       holtzmann.xPos < shield.xPos + SHIELD_LENGTH/2 &&
       holtzmann.xPos > shield.xPos - SHIELD_LENGTH/2 &&
       holtzmann.yPos - HM_DIAM/2 < shield.yPos + SHIELD_HEIGHT/2)
    {
       holtzmann.yVel = -holtzmann.yVel + 1;
       holtzmann.yPos = shield.yPos - SHIELD_HEIGHT/2 - HM_DIAM/2;
    }

    // post on shield data mutex
    OSMutexPost (&shield_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // check if holtzmann mass touching wall -> if so reverse its x velocity and make x acceleration 0
    if(holtzmann.xPos - HM_DIAM/2 < left_wall_x) {
        holtzmann.xVel = -holtzmann.xVel;
        holtzmann.xAccl = 0;
        holtzmann.xPos = left_wall_x + HM_DIAM/2;
    }

    if(holtzmann.xPos + HM_DIAM/2 > right_wall_x) {
        holtzmann.xVel = -holtzmann.xVel;
        holtzmann.xAccl = 0;
        holtzmann.xPos = right_wall_x - 1 - HM_DIAM/2;
    }

    // if holtzmann mass goes off bottom of screen
    if(holtzmann.yPos > 150) {
      holtzmann.yPos = 0;
      holtzmann.yVel = 0;
    }

    // post on holtzmann data mutex
    OSMutexPost (&holtzmann_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  }
}

/***************************************************************************//**
 * LED0 Update Task.
 ******************************************************************************/
static void led0_update_task(void *arg) {

}

/***************************************************************************//**
 * LED1 Update Task.
 ******************************************************************************/
static void led1_update_task(void *arg) {

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
        OSTimeDly(60, OS_OPT_TIME_DLY, &err);

        // reset the screen
        GLIB_clear(&glibContext);

        int shield_xPos = SHIELD_INITIAL_X;
        int shield_yPos = SHIELD_INITIAL_Y;

        int holtzmann_xPos = HM_INITIAL_X_POSITION;
        int holtzmann_yPos = HM_INITIAL_Y_POSITION;

        // pend on shield data mutex
        OSMutexPend(&shield_mutex,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    0,
                    &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // get shield data
        shield_xPos = shield.xPos;

        // post on shield data mutex
        OSMutexPost (&shield_mutex,
                     OS_OPT_POST_NONE,
                     &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // pend on holtzmann data mutex
        OSMutexPend(&holtzmann_mutex,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    0,
                    &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        holtzmann_xPos = holtzmann.xPos;
        holtzmann_yPos = holtzmann.yPos;

        // post on holtzmann data mutex
        OSMutexPost (&holtzmann_mutex,
                     OS_OPT_POST_NONE,
                     &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        draw_holtzmann(holtzmann_xPos, holtzmann_yPos, &glibContext);
        draw_shield(shield_xPos, shield_yPos, &glibContext);
        draw_canyon(&glibContext);

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

  if(far_left && !right && !far_right)
    slider_dir = HARD_LEFT;
  else if (left && !right && !far_right)
    slider_dir = SOFT_LEFT;
  else if (!far_left && !left && right)
    slider_dir = SOFT_RIGHT;
  else if (!far_left && !left && far_right)
    slider_dir = HARD_RIGHT;
  else
    slider_dir = STRAIGHT;

  return slider_dir;
}

/***************************************************************************//**
 * @brief
 *   Draw holtzmann mass on LCD screen at given xPos and yPos.
 ******************************************************************************/
void draw_holtzmann(int xPos, int yPos, GLIB_Context_t * glibContext) {

  GLIB_drawCircleFilled(glibContext,
                        xPos,
                        yPos,
                        HM_DIAM/2);
}

/***************************************************************************//**
 * @brief
 *   Draw shield platform on LCD screen at given xPos and yPos.
 ******************************************************************************/
void draw_shield(int xPos, int yPos, GLIB_Context_t * glibContext) {

  GLIB_Rectangle_t shield_platform;
  shield_platform.xMax = xPos + SHIELD_LENGTH/2;
  shield_platform.xMin = xPos - SHIELD_LENGTH/2;
  shield_platform.yMax = yPos + SHIELD_HEIGHT/2;
  shield_platform.yMin = yPos - SHIELD_HEIGHT/2;

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

  left_wall.xMax = left_wall_x;
  left_wall.xMin = 0;
  left_wall.yMax = 128;
  left_wall.yMin = 0;

  right_wall.xMax = 128;
  right_wall.xMin = right_wall_x;
  right_wall.yMax = 128;
  right_wall.yMin = 0;

  GLIB_drawRectFilled(glibContext,
                      &left_wall);

  GLIB_drawRectFilled(glibContext,
                      &right_wall);
}
