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
#include "town.h"
#include "person2.h"

// -------------------------------------------------------------------------- //
// ------------------ FINAL PROJECT STUFF ----------------------------------- //
// -------------------------------------------------------------------------- //

  // LCD Display Task
#define LCD_TASK_STACK_SIZE        256
#define LCD_TASK_PRIO              18
  // LaserUpdateTask
#define LASER_TASK_STACK_SIZE        256
#define LASER_TASK_PRIO              16
  // ShieldEnhanceTask
#define SHIELD_ENHANCE_TASK_STACK_SIZE        512
#define SHIELD_ENHANCE_TASK_PRIO              15
  // ShieldForceTask
#define SHIELD_FORCE_TASK_STACK_SIZE        512
#define SHIELD_FORCE_TASK_PRIO              14
  // PhysicsUpdateTask
#define PHYSICS_TASK_STACK_SIZE        512
#define PHYSICS_TASK_PRIO              17
  // LED0Task
#define LED0_TASK_STACK_SIZE        256
#define LED0_TASK_PRIO              19
  // LED1Task
#define LED1_TASK_STACK_SIZE        256
#define LED1_TASK_PRIO              20

// Game Element Dimensions
#define SHIELD_WIDTH 40000
#define SHIELD_HEIGHT 2000

// -------------------------- Project Config Values ------------------- //
// Game Rules
#define DATA_STRUCTURE_VERSION 4
#define TAU_PHYSICS 10 // ms
#define TAU_LCD 60 // ms
#define GRAVITY -98000 // mm/s^2
#define CANYON_SIZE 100000 // cm
// Holtzmann Rules
#define HM_NUM 3 // number of holtzmann masses
#define HM_DIAM 10000 // cm
#define HM_INIT_CONDITIONS 0 //enum: Fixed=0. 127+ available for user defined modes
#define HM_INITIAL_X_VELOCITY 4000 // cm/s
#define HM_INITIAL_Y_VELOCITY 0 // cm/s
#define HM_INITIAL_X_POSITION 0 // cm
#define HM_USER_DEFINED_MODE_INPUT 0 //i.e. 8x32b set aside for user-defined mode params
// Shield Rules
#define SHIELD_MAX_FORCE 200000 // N
#define SHIELD_MASS 100 // kg
#define SHIELD_LENGTH 15000 // cm
#define SHIELD_BOUNCE_ENABLED 1 // T/F
#define SHIELD_BOUNCE_LIMITED 0 // T/F
#define SHIELD_MAX_BOUNCE_SPEED 0 // cm/s
#define SHIELD_AUTO_CONTROL 0 // T/F
#define SHIELD_MIN_PERPENDICULAR_SPEED 0 // cm/s
#define SHIELD_ENERGY_REDUCTION 70 //exclusively passive bounce kinetic energy reduction %
#define SHIELD_ENERGY_INCREASE 40 //kinetic energy increase %
#define SHIELD_ARMING_WINDOW 500 //ms
#define SHIELD_RECHARGE_TIME 1000 //ms
// Laser Rules
#define NUM_LASERS 1
#define AUTO_LASERS 0 //T/F
// -------------------------------------------------------------------------------------- //

// Stuff not in the project config values
// LED Initial Periods
#define LED1_INITIAL_PERIOD 2 // ms
#define LED1_INITIAL_ONESHOT 1 // ms

#define LED0_INITIAL_PERIOD 2 // ms
#define LED0_INITIAL_ONESHOT 1 // ms

#define LED3_INITIAL_PERIOD 80
#define LED3_INITIAL_ONESHOT 40

#define SHIELD_INITIAL_X 0
#define SHIELD_INITIAL_Y 10000

#define HM_INITIAL_Y_POSITION 100000 // cm
#define HM_IMAGE 0 // 0 is just a sphere that can change size, 1 is a little person
#define HM_MASS 100 // kg
#define HM_MIN_SPEED 20000 // cm/s

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the shield data.
//----------------------------------------------------------------------------------------------------------------------------------
struct shield_data {
  int xPos;
  int yPos;
  int xVel;
  int xAccl;
  int forceApplied;
  int enhanced;
  int timeEnhanced;
  int timeUnenhanced;
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

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the game data.
//----------------------------------------------------------------------------------------------------------------------------------
struct game_data {
  int game_over;
  int num_HM_left;
  int laser_active;
  int num_laser_left;
};

/***************************************************************************//**
 * Initialize blink example
 ******************************************************************************/
void blink_init(void);

#endif  // BLINK_H
