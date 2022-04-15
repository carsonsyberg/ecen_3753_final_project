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
                                    .yForce = GRAVITY,
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
// BTN0 Semaphore
static OS_SEM btn0_semaphore;
// BTN1 Semaphore
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

enum direction_vals {STRAIGHT = 0, HARD_LEFT = 1, SOFT_LEFT = 2, SOFT_RIGHT = 3, HARD_RIGHT = 4};
enum touching_wall {LEFT_WALL = 0, RIGHT_WALL = 1, NO_WALL = -1};
enum event_flags {BUTTON_DOWN = 1, BUTTON_UP = 2}; // need to be 1 and 2 since they are bits 0001, 0010
enum led0_flags {GAME_OVER = 1, FORCE_REQUIRED = 2};
enum led1_flags {GAME_OVER = 1, FORCE_UPDATE = 2};

// LCD Task Stuff
static GLIB_Context_t glibContext;
static volatile uint32_t msTicks = 0;

// Canyon Walls Stuff
#define LEFT_WALL_X (128 - CANYON_SIZE)/2
#define RIGHT_WALL_X (128 - CANYON_SIZE)/2 + CANYON_SIZE

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
static void idle_task(void *arg);
static void lcd_output_task(void *arg);
void laser_valid(int num_lasers, int * data);
int shield_touching_wall(int shieldX);
int hm_touching_shield(int hmX, int hmY, int shieldX, int shieldY);
int hm_touching_wall(int hmX);
void update_hm_x_physics(int xForce, int xAccl, int xVel, int xPos, int touchingWall, int * hm_data);
void update_hm_y_physics(int yForce, int yAccl, int yVel, int yPos, int shieldY, int touchingShield, int shieldEnhanced, int * hm_data);
void update_shield_physics(int force, int xVel, int xAccl, int xPos, int touchingWall, int * shield_data);

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
//  read_button0();
  RTOS_ERR err;

  // Post BTN0 Semaphore
  OSSemPost (&btn0_semaphore,
             OS_OPT_POST_ALL,
             &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

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

  // Post BTN0 Event Flag (depending on button up or button down)
  if(!btn1_state) {
    OSFlagPost (&btn1_flag_group,
                BUTTON_DOWN,
                OS_OPT_POST_FLAG_SET,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }
  if(btn1_state && first_interrupt != 0){
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

  // ------------------ CREATE FINAL PROJECT DATA STRUCTURES ---------------- //
  // BTN0 Semaphore
  OSSemCreate(&btn0_semaphore,
              "BTN0 Semaphore",
              0,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  // BTN1 Flag Groupo
  OSFlagCreate(&btn1_flag_group,
               "BTN1 Pressed/Unpressed Flags",
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
  RTOS_ERR err;
  PP_UNUSED_PARAM(arg);

  while(1) {
      // call OS function to pend on BTN0 semaphore
      OSSemPend(&btn0_semaphore,
                 0,
                 OS_OPT_PEND_BLOCKING,
                 NULL,
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

      int laser_active[2];
      laser_valid(game.num_laser_left, laser_active);
      game.num_laser_left = laser_active[1];
      game.laser_active = laser_active[0];

      // post on shield data mutex
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
                             BUTTON_UP | BUTTON_DOWN,
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
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

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
      OSTimeDly(20, OS_OPT_TIME_DLY, &err);
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

      // Post LED1 flag
      OSFlagPost (&led1_flag_group,
                  FORCE_UPDATE,
                  OS_OPT_POST_FLAG_SET,
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
    OSTimeDly(100, OS_OPT_TIME_DLY, &err);

    // pend on shield data mutex
    OSMutexPend(&shield_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // check if touching a wall -> if so reverse its velocity and make acceleration 0
    int shield_collides_wall = shield_touching_wall(shield.xPos);
    int shield_physics_data[4];
    update_shield_physics(shield.forceApplied, shield.xVel, shield.xAccl, shield.xPos, shield_collides_wall, shield_physics_data);

    shield.forceApplied = shield_physics_data[0];
    shield.xAccl = shield_physics_data[1];
    shield.xVel = shield_physics_data[2];
    shield.xPos = shield_physics_data[3];

    // pend on holtzmann data mutex
    OSMutexPend(&holtzmann_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    int hm_collides_shield = hm_touching_shield(holtzmann.xPos, holtzmann.yPos, shield.xPos, shield.yPos);
    // once hm_collides_shield is true, we can check if the shield is still enhanced
    if(hm_collides_shield) {
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
    int hm_y_physics_data[3];
    update_hm_y_physics(holtzmann.yForce, holtzmann.yAccl, holtzmann.yVel, holtzmann.yPos, shield.yPos, hm_collides_shield, shield.enhanced, hm_y_physics_data);

    holtzmann.yAccl = hm_y_physics_data[0];
    holtzmann.yVel = hm_y_physics_data[1];
    holtzmann.yPos = hm_y_physics_data[2];

    // post on shield data mutex
    OSMutexPost (&shield_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // check if holtzmann mass touching wall -> if so reverse its x velocity and make x acceleration 0
    int hm_collides_wall = hm_touching_wall(holtzmann.xPos);
    int hm_x_physics_data[3];
    update_hm_x_physics(holtzmann.xForce, holtzmann.xAccl, holtzmann.xVel, holtzmann.xPos, hm_collides_wall, hm_x_physics_data);

    holtzmann.xAccl = hm_x_physics_data[0];
    holtzmann.xVel = hm_x_physics_data[1];
    holtzmann.xPos = hm_x_physics_data[2];

    // if holtzmann mass goes off bottom of screen
    if(holtzmann.yPos > 150) {
      holtzmann.yPos = 0;
      holtzmann.yVel = 0;
    }

    // pend on game data mutex
    OSMutexPend(&game_mutex,
                0,
                OS_OPT_PEND_BLOCKING,
                0,
                &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // update game data
    if(game.laser_active) {
        game.num_HM_left--; // one HM destroyed
        holtzmann.yPos = 0; // reset HM position
        holtzmann.yVel = 0; // and velocity
        game.laser_active = 0;
    }


    if(holtzmann.yPos < SHIELD_INITIAL_Y - 0.5*SHIELD_HEIGHT) {
        game.game_over = 1;

        // Post LED0 flag
        OSFlagPost (&led0_flag_group,
                    GAME_OVER,
                    OS_OPT_POST_FLAG_SET,
                    &err);
    }
    else {
        // Post LED0 flag
        OSFlagPost (&led0_flag_group,
                    FORCE_REQUIRED,
                    OS_OPT_POST_FLAG_SET,
                    &err);
    }

    // post on holtzmann data mutex
    OSMutexPost (&holtzmann_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    // post on shield data mutex
    OSMutexPost (&game_mutex,
                 OS_OPT_POST_NONE,
                 &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));



  }
}

//static void physics_update_task(void *arg) {
//  PP_UNUSED_PARAM(arg);
//  RTOS_ERR err;
//  // update the position, velocity, and acceleration of the Shield Platform
//    // if ax = 0, velocity unchanged, else velocity changes
//    // if vx = 0, position unchanged, else position changes
//    // if finger on slider, constant acceleration applied
//    // if platform collides with wall,
//  // update the position, velocity, and acceleration of the Holtzmann Mass
//
//  // infinite while loop
//  while (1)
//  {
//    // call OSTimeDly to delay between each loop iteration
//    OSTimeDly(120, OS_OPT_TIME_DLY, &err);
//
//    // pend on shield data mutex
//    OSMutexPend(&shield_mutex,
//                0,
//                OS_OPT_PEND_BLOCKING,
//                0,
//                &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    // update shield data
//    shield.xAccl = shield.forceApplied/SHIELD_MASS;
//    shield.xVel = shield.xVel + shield.xAccl;
//    shield.xPos = shield.xPos + shield.xVel;
//
//    // check if touching a wall -> if so reverse its velocity and make acceleration 0
//    if(shield.xPos - SHIELD_LENGTH/2 < LEFT_WALL_X) {
//        if(SHIELD_BOUNCE_ENABLED)
//          shield.xVel = -shield.xVel - 1;
//        else
//          shield.xVel = 0;
//        shield.xAccl = 0;
//        shield.forceApplied = 0;
//        shield.xPos = LEFT_WALL_X + 1 + SHIELD_LENGTH/2;
//    }
//
//    if(shield.xPos + SHIELD_LENGTH/2 > RIGHT_WALL_X) {
//        if(SHIELD_BOUNCE_ENABLED)
//          shield.xVel = -shield.xVel + 1;
//        else
//          shield.xVel = 0;
//        shield.xAccl = 0;
//        shield.forceApplied = 0;
//        shield.xPos = RIGHT_WALL_X - 1 - SHIELD_LENGTH/2;
//    }
//
//    // pend on holtzmann data mutex
//    OSMutexPend(&holtzmann_mutex,
//                0,
//                OS_OPT_PEND_BLOCKING,
//                0,
//                &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    // update holtzmann data
//    holtzmann.xAccl = holtzmann.xForce/HM_MASS;
//    holtzmann.yAccl = holtzmann.yForce/HM_MASS;
//    holtzmann.xVel = holtzmann.xVel + holtzmann.xAccl;
//    holtzmann.yVel = holtzmann.yVel + holtzmann.yAccl;
//    holtzmann.xPos = holtzmann.xPos + holtzmann.xVel;
//    holtzmann.yPos = holtzmann.yPos + holtzmann.yVel;
//
//    // check if holtzmann mass touching platform
//      // -> if so
//    if(holtzmann.yPos + HM_DIAM/2 > shield.yPos - SHIELD_HEIGHT/2 &&
//       holtzmann.xPos < shield.xPos + SHIELD_LENGTH/2 &&
//       holtzmann.xPos > shield.xPos - SHIELD_LENGTH/2 &&
//       holtzmann.yPos - HM_DIAM/2 < shield.yPos + SHIELD_HEIGHT/2)
//    {
//
//       holtzmann.yVel = -holtzmann.yVel + 1;
//       holtzmann.yPos = shield.yPos - SHIELD_HEIGHT/2 - HM_DIAM/2;
//       // -> go through the shield IF HM velocity is too slow holtzmann.yVel < HM_MIN_SPEED
////       if(holtzmann.yVel < HM_MIN_SPEED) {
////           holtzmann.yPos =
////       }
//       // -> reverse and increase magnitude of y velocity IF enhanced shield by SHIELD_ENERGY_INCREASE %
//         // -> shield is enhanced IF currentTime - shield.timeEnhanced <= SHIELD_ARMING_WINDOW
//       // -> reverse its y velocity IF not enhanced shield by SHIELD_ENERGY_REDUCTION % KE = 0.5*m*v^2 -> v = sqrt(2*KE/m)
//    }
//
//    // post on shield data mutex
//    OSMutexPost (&shield_mutex,
//                 OS_OPT_POST_NONE,
//                 &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    // check if holtzmann mass touching wall -> if so reverse its x velocity and make x acceleration 0
//    if(holtzmann.xPos - HM_DIAM/2 < LEFT_WALL_X) {
//        holtzmann.xVel = -holtzmann.xVel;
//        holtzmann.xAccl = 0;
//        holtzmann.xPos = LEFT_WALL_X + HM_DIAM/2;
//    }
//
//    if(holtzmann.xPos + HM_DIAM/2 > RIGHT_WALL_X) {
//        holtzmann.xVel = -holtzmann.xVel;
//        holtzmann.xAccl = 0;
//        holtzmann.xPos = RIGHT_WALL_X - 1 - HM_DIAM/2;
//    }
//
//    // if holtzmann mass goes off bottom of screen
//    if(holtzmann.yPos > 150) {
//      holtzmann.yPos = 0;
//      holtzmann.yVel = 0;
//    }
//
//    // pend on game data mutex
//    OSMutexPend(&game_mutex,
//                0,
//                OS_OPT_PEND_BLOCKING,
//                0,
//                &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    // update game data
//    if(game.laser_active) {
//        game.num_HM_left--; // one HM destroyed
//        holtzmann.yPos = 0; // reset HM position
//        holtzmann.yVel = 0; // and velocity
//        game.laser_active = 0;
//    }
//
//    if(holtzmann.yPos < SHIELD_INITIAL_Y - 0.5*SHIELD_HEIGHT) {
//        game.game_over = 1;
//    }
//
//    // post on holtzmann data mutex
//    OSMutexPost (&holtzmann_mutex,
//                 OS_OPT_POST_NONE,
//                 &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    // post on shield data mutex
//    OSMutexPost (&game_mutex,
//                 OS_OPT_POST_NONE,
//                 &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//
//  }
//}

// returns 0 for left wall and 1 for right wall and -1 for niether
int shield_touching_wall(int shieldX) {
  // check if touching a wall -> if so reverse its velocity and make acceleration 0
  if(shieldX - SHIELD_LENGTH/2 < LEFT_WALL_X) {
      return LEFT_WALL;
  }
  else if(shieldX + SHIELD_LENGTH/2 > RIGHT_WALL_X) {
      return RIGHT_WALL;
  }
  else {
      return NO_WALL;
  }
}

int hm_touching_wall(int hmX) {
  // check if holtzmann mass touching wall -> if so reverse its x velocity and make x acceleration 0
  if(hmX - HM_DIAM/2 < LEFT_WALL_X) {
      return LEFT_WALL;
  }
  else if(hmX + HM_DIAM/2 > RIGHT_WALL_X) {
      return RIGHT_WALL;
  }
  else {
      return NO_WALL;
  }
}

int hm_touching_shield(int hmX, int hmY, int shieldX, int shieldY) {
  // check if holtzmann mass touching platform
  if(hmY + HM_DIAM/2 > shieldY - SHIELD_HEIGHT/2 &&
      hmX < shieldX + SHIELD_LENGTH/2 &&
      hmX > shieldX - SHIELD_LENGTH/2 &&
      hmY - HM_DIAM/2 < shieldY + SHIELD_HEIGHT/2)
  {
      return 1;
  }
  else {
      return 0;
  }
}

void update_hm_x_physics(int xForce, int xAccl, int xVel, int xPos, int touchingWall, int * hm_data) {

  // not touching a wall, business as usual
  hm_data[1] = xVel + xAccl; // xVel
  hm_data[0] = xForce/HM_MASS; // xAccl
  hm_data[2] = xPos + xVel; // xPos

  // check if holtzmann mass touching wall -> if so reverse its x velocity and make x acceleration 0
  if(touchingWall == LEFT_WALL) {
    hm_data[1] = -xVel; // xVel
    hm_data[0] = 0; // xAccl
    hm_data[2] = LEFT_WALL_X + HM_DIAM/2; // xPos
  }
  else if(touchingWall == RIGHT_WALL) {
    hm_data[1] = -xVel; // xVel
    hm_data[0] = 0; // xAccl
    hm_data[2] = RIGHT_WALL_X - 1 - HM_DIAM/2; // xPo
  }

}

void update_hm_y_physics(int yForce, int yAccl, int yVel, int yPos, int shieldY, int touchingShield, int shieldEnhanced, int * hm_data) {

  // not touching platform, just keeps falling
  hm_data[0] = yForce/HM_MASS; // yAccl
  hm_data[1] = yVel + yAccl; // yVel
  hm_data[2] = yPos + yVel; // yPos

  // Y physics depends on touching platform or not && on the boost applied from enhanced shield
  if(touchingShield && yVel >= HM_MIN_SPEED) {
    // HM WILL BOUNCE OFF THE SHIELD
    if(shieldEnhanced) {
    // -> reverse and increase magnitude of y velocity IF enhanced shield by SHIELD_ENERGY_INCREASE %
      // need to know KE before impact, then calculate new KE w/ % increase, then calculate new v in opposite direction
//      int double_energy_pre_impact = HM_MASS*(yVel*yVel);
//      int double_energy_post_impact = double_energy_pre_impact - (int)((SHIELD_ENERGY_INCREASE/100)*double_energy_pre_impact);
      hm_data[1] = -yVel - SHIELD_ENERGY_INCREASE; // yVel
      hm_data[2] = shieldY - SHIELD_HEIGHT/2 - HM_DIAM/2; // yPos
      hm_data[0] = 0; // yAccl
    }
    else {
    // -> reverse its y velocity IF not enhanced shield by SHIELD_ENERGY_REDUCTION % KE = 0.5*m*v^2 -> v = sqrt(2*KE/m)
      // need to know KE before impact, then calculate new KE w/ % increase, then calculate new v in opposite direction
//        int double_energy_pre_impact = HM_MASS*(yVel*yVel);
//        int double_energy_post_impact = double_energy_pre_impact - (int)((SHIELD_ENERGY_REDUCTION/100)*double_energy_pre_impact);
        hm_data[1] = -yVel + SHIELD_ENERGY_REDUCTION; // yVel
        hm_data[2] = shieldY - SHIELD_HEIGHT/2 - HM_DIAM/2; // yPos
        hm_data[0] = 0; // yAccl
    }
  }
  else if(touchingShield && yVel < HM_MIN_SPEED) {
      hm_data[2] = shieldY + SHIELD_HEIGHT/2 + HM_DIAM/2; // yPos
  }

//  // not touching platform, just keeps falling
//  hm_data[0] = yForce/HM_MASS; // yAccl
//  hm_data[1] = yVel + yAccl; // yVel
//  hm_data[2] = yPos + yVel; // yPos
//
//  // Y physics depends on touching platform or not && on the boost applied from enhanced shield
//  if(touchingShield && yVel >= HM_MIN_SPEED) {
//    hm_data[1] = -yVel + 3; // yVel
//    hm_data[2] = shieldY - SHIELD_HEIGHT/2 - HM_DIAM/2; // yPos
//    hm_data[0] = 0; // yAccl
//    // -> reverse its y velocity IF not enhanced shield by SHIELD_ENERGY_REDUCTION % KE = 0.5*m*v^2 -> v = sqrt(2*KE/m)
//  }
//  else if(touchingShield && yVel < HM_MIN_SPEED) {
//      hm_data[2] = shieldY + SHIELD_HEIGHT/2 + HM_DIAM/2; // yPos
//  }
//
//  // -> reverse and increase magnitude of y velocity IF enhanced shield by SHIELD_ENERGY_INCREASE %
//  // -> shield is enhanced IF currentTime - shield.timeEnhanced <= SHIELD_ARMING_WINDOW

}

void update_shield_physics(int force, int xVel, int xAccl, int xPos, int touchingWall, int * shield_data) {

  // means shield is not colliding with wall, just give it regular old physics
  shield_data[1] = force/SHIELD_MASS; // xAccl
  shield_data[2] = xVel + xAccl; // xVel
  shield_data[3] = xPos + xVel; // xPos
  shield_data[0] = force;       // forceApplied

  // include wall touching logic that changes shield physics stuff
  // check if touching a wall -> if so reverse its velocity and make acceleration 0
  if(touchingWall == LEFT_WALL) {
      if(SHIELD_BOUNCE_ENABLED)
        shield_data[2] = -xVel - 1; // reverse velocity (and take away some speed)
      else
        shield_data[2] = 0; // reverse velocity (leave speed intact)
      shield_data[1] = 0; // xAccl = 0
      shield_data[0] = 0; // force = 0
      shield_data[3] = LEFT_WALL_X + 1 + SHIELD_LENGTH/2; // xPos
  }
  else if(touchingWall == RIGHT_WALL) {
      if(SHIELD_BOUNCE_ENABLED)
        shield_data[2] = -xVel + 1; // xVel reversed (less speed)
      else
        shield_data[2] = 0; // xVel reversed (same speed)
      shield_data[1] = 0; // xAccl = 0
      shield_data[0] = 0; // force = 0
      shield_data[3] = RIGHT_WALL_X - 1 - SHIELD_LENGTH/2; // xPos
  }
}

/***************************************************************************//**
 * LED0 Update Task.
 ******************************************************************************/
static void led0_update_task(void *arg) {
  // Normally this is used to provide simple guidance to your platform operator.
  // Assuming that the HM’s current trajectory and the platform’s trajectory would
  // not hit a canyon wall, pulse width modulate this LED to show the % of MAX_FORCE
  // that would need to be applied to the platform to get it to intersect the HM center-on.

  //

  // If an HM has gotten through the shield or onto the ground thereby leaving the base open
  // to the Harkonnen, blink on/off with a 50% duty cycle at 1Hz to alert all nearby personnel to evacuate.

}

/***************************************************************************//**
 * LED1 Update Task.
 ******************************************************************************/
static void led1_update_task(void *arg) {
  // Pulse width modulated to show (via human-perceived brightness) the current force magnitude, as a % of MAX_FORCE.
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
        int shield_enhanced = 0;

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
        shield_enhanced = shield.enhanced;

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

        draw_town(&glibContext);
        draw_holtzmann(holtzmann_xPos, holtzmann_yPos, &glibContext);
        draw_shield(shield_xPos, shield_yPos, &glibContext);
        draw_canyon(&glibContext);

        // DEBUG PRINT ---------------------------------------------------------------
        GLIB_drawString (&glibContext,
                         &print_string,
                         strlen(print_string),
                         5,
                         5,
                         true
        );

        if(shield_enhanced) {
            GLIB_drawString (&glibContext,
                             "EN",
                             strlen("EN"),
                             5,
                             15,
                             true
            );
        }
        else {
            GLIB_drawString (&glibContext,
                             "DIS",
                             strlen("DIS"),
                             5,
                             15,
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

  if(HM_IMAGE == 0) {
    GLIB_drawCircleFilled(glibContext,
                          xPos,
                          yPos,
                          HM_DIAM/2);
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

/***************************************************************************//**
 * @brief
 *   Sample pushbutton 0 only if pushbutton 1 is not currently pressed.
 ******************************************************************************/
void read_buttons() {
  // 0 if pressed, 1 not pressed GPIO_PinInGet
  uint32_t btn0_state = GPIO_PinInGet(BUTTON0_port, BUTTON0_pin);
  uint32_t btn1_state = GPIO_PinInGet(BUTTON1_port, BUTTON1_pin);
}
