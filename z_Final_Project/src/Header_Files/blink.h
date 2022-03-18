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

#ifndef BLINK_H
#define BLINK_H

#include "gpio.h"
#include "capsense.h"
#include "em_emu.h"
#include "app.h"

//#define BLINK_TASK_STACK_SIZE      96
//#define BLINK_TASK_PRIO            20

#define BUTTON_TASK_STACK_SIZE     256
#define BUTTON_TASK_PRIO           18

#define SLIDER_TASK_STACK_SIZE     256
#define SLIDER_TASK_PRIO           18

#define IDLE_TASK_STACK_SIZE        256
#define IDLE_TASK_PRIO              21

  // Speed Setpoint Task
#define SPEED_TASK_STACK_SIZE        256
#define SPEED_TASK_PRIO              19
  // Vehicle Direction Task
#define DIR_TASK_STACK_SIZE        256
#define DIR_TASK_PRIO              19
  // Vehicle Monitor Task
#define MONITOR_TASK_STACK_SIZE        256
#define MONITOR_TASK_PRIO              20
  // LED Output Task
#define LED_TASK_STACK_SIZE        256
#define LED_TASK_PRIO              17

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the vehicle speed data.
//----------------------------------------------------------------------------------------------------------------------------------
struct speed_data {
  // current speed
  // count of speed increments
  // count of speed decrements
  int curr_speed;
  int incr_count;
  int decr_count;
};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the vehicle direction data.
//----------------------------------------------------------------------------------------------------------------------------------
struct direction_data {
  // current direction
  // time without direction changing
  // count of left turns
  // count of right turns
  int curr_dir;
  int time_since_change;
  int left_count;
  int right_count;
};

// -------------------------------------------------------------------------- //
// ------------------ FINAL PROJECT STUFF ----------------------------------- //
// -------------------------------------------------------------------------- //

  // LCD Display Task
#define LCD_TASK_STACK_SIZE        256
#define LCD_TASK_PRIO              19
  // LaserUpdateTask
#define LASER_TASK_STACK_SIZE        256
#define LASER_TASK_PRIO              20
  // ShieldEnhanceTask
#define SHIELD_ENHANCE_TASK_STACK_SIZE        256
#define SHIELD_ENHANCE_TASK_PRIO              20
  // ShieldForceTask
#define SHIELD_FORCE_TASK_STACK_SIZE        256
#define SHIELD_FORCE_TASK_PRIO              17
  // PhysicsUpdateTask
#define PHYSICS_TASK_STACK_SIZE        256
#define PHYSICS_TASK_PRIO              18
  // LED0Task
#define LED0_TASK_STACK_SIZE        256
#define LED0_TASK_PRIO              20
  // LED1Task
#define LED1_TASK_STACK_SIZE        256
#define LED1_TASK_PRIO              20

// Game Element Dimensions
#define SHIELD_WIDTH 40
#define SHIELD_HEIGHT 6

// -------------------------- YOU CHOOSE THE BELOW VALUES ------------------- //
// -------------------------- USE WHATEVER WORKS ---------------------------- //

// Game Rules
#define DATA_STRUCTURE_VERSION 1
#define GRAVITY 10 //mm/s^2
#define CANYON_SIZE 100 //cm (px)
// Holtzmann Rules
#define HM_NUM 10 // number of holtzmann masses
#define HM_DIAM 10 //cm
#define HM_INIT_CONDITIONS 1 //enum: Fixed=0. 127+ available for user defined modes
#define HM_INITIAL_X_VELOCITY 5 //cm/s
#define HM_INITIAL_Y_VELOCITY 0 //cm/s
#define HM_INITIAL_X_POSITION 64 //mm
#define HM_INITIAL_Y_POSITION 0
#define HM_USER_DEFINED_MODE_INPUT 0 //i.e. 8x32b set aside for user-defined mode params
#define HM_MASS 10
// Shield Rules
#define SHIELD_INITIAL_X 64
#define SHIELD_INITIAL_Y 119
#define SHIELD_MAX_FORCE 20 //N
#define SHIELD_MASS 10 //kg
#define SHIELD_LENGTH 40 //cm
#define SHIELD_BOUNCE_ENABLED 1 //T/F
#define SHIELD_BOUNCE_LIMITED 0 //T/F
#define SHIELD_MAX_BOUNCE_SPEED //cm/s
#define SHIELD_AUTO_CONTROL 0 //T/F
#define SHIELD_ENERGY_REDUCTION 0 //exclusively passive bounce kinetic energy reduction %
#define SHIELD_ENERGY_INCREASE 0 //kinetic energy increase %
#define SHIELD_ARMING_WINDOW 0 //ms
#define SHIELD_RECHARGE_TIME 0 //ms
// Laser Rules
#define NUM_LASERS 10
#define AUTO_LASERS 0 //T/F

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the shield data.
//----------------------------------------------------------------------------------------------------------------------------------
struct shield_data {
  int xPos;
  int yPos;
  int xVel;
  int xAccl;
  int forceApplied;
};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the holtzmann mass data.
//----------------------------------------------------------------------------------------------------------------------------------
struct holtzmann_data {
  int xPos;
  int yPos;
  int xVel;
  int yVel;
  int xAccl;
  int yAccl;
  int xForce;
  int yForce;
};
/***************************************************************************//**
 * Initialize blink example
 ******************************************************************************/
void blink_init(void);

#endif  // BLINK_H
