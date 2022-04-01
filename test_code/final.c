#include "final.h"
#include <stdio.h>
#include <math.h>

// init
void init(
  int shield_touching_wall, int hm_touching_wall, int hm_touching_shield, 
  int hm_x_data[3], int hm_y_data[3], int shield_data[3], int laser_data[2],
  int touching[3], int hm_x[3], int hm_y[3], int shield[3], int laser[2]) 
{
  shield_touching_wall = touching[0];
  hm_touching_wall = touching[1];
  hm_touching_shield = touching[2];

  hm_x_data[0] = hm_x[0];
  hm_x_data[1] = hm_x[1];
  hm_x_data[2] = hm_x[2];

  hm_y_data[0] = hm_y[0];
  hm_y_data[1] = hm_y[1];
  hm_y_data[2] = hm_y[2];

  shield_data[0] = shield[0];
  shield_data[1] = shield[1];
  shield_data[2] = shield[2];

  laser_data[0] = laser[0];
  laser_data[1] = laser[1];
}

// returns 0 for left wall and 1 for right wall and -1 for niether
int shield_touching_wall(int shieldX, int SHIELD_LENGTH, int LEFT_WALL_X, int RIGHT_WALL_X) {
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

int hm_touching_wall(int hmX, int HM_DIAM, int LEFT_WALL_X, int RIGHT_WALL_X) {
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

int hm_touching_shield(int hmX, int hmY, int shieldX, int shieldY, int SHIELD_HEIGHT, int SHIELD_LENGTH, int HM_DIAM) {
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

void update_hm_x_physics(int xForce, int xAccl, int xVel, int xPos, int touchingWall, int HM_MASS, int HM_DIAM, int LEFT_WALL_X, int RIGHT_WALL_X, int * hm_data) {

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

void update_hm_y_physics(int yForce, int yAccl, int yVel, int yPos, int shieldY, int touchingShield, int shield_enhanced, int HM_MASS, int HM_DIAM, int HM_MIN_SPEED, int SHIELD_HEIGHT, int SHIELD_ENERGY_INCREASE, int SHIELD_ENERGY_REDUCTION, int * hm_data) {

  // not touching platform, just keeps falling
  hm_data[0] = yForce/HM_MASS; // yAccl
  hm_data[1] = yVel + yAccl; // yVel
  hm_data[2] = yPos + yVel; // yPos

  // Y physics depends on touching platform or not && on the boost applied from enhanced shield
  if(touchingShield && yVel >= HM_MIN_SPEED) {
    // HM WILL BOUNCE OFF THE SHIELD
    if(shield_enhanced) {
    // -> reverse and increase magnitude of y velocity IF enhanced shield by SHIELD_ENERGY_INCREASE %
      // need to know KE before impact, then calculate new KE w/ % increase, then calculate new v in opposite direction
      int energy_pre_impact = 0.5*HM_MASS*(yVel*yVel);
      int energy_post_impact = energy_pre_impact + SHIELD_ENERGY_INCREASE*energy_pre_impact;
      hm_data[1] = sqrt((2*energy_post_impact)/HM_MASS); // -yVel + 3; // yVel
      hm_data[2] = shieldY - SHIELD_HEIGHT/2 - HM_DIAM/2; // yPos
      hm_data[0] = 0; // yAccl
    }
    else {
    // -> reverse its y velocity IF not enhanced shield by SHIELD_ENERGY_REDUCTION % KE = 0.5*m*v^2 -> v = sqrt(2*KE/m)
      // need to know KE before impact, then calculate new KE w/ % increase, then calculate new v in opposite direction
      int energy_pre_impact = 0.5*HM_MASS*(yVel*yVel);
      int energy_post_impact = energy_pre_impact - SHIELD_ENERGY_REDUCTION*energy_pre_impact;
      hm_data[1] = sqrt((2*energy_post_impact)/HM_MASS); //-yVel + 3; // yVel
      hm_data[2] = shieldY - SHIELD_HEIGHT/2 - HM_DIAM/2; // yPos
      hm_data[0] = 0; // yAccl
    }
  }
  else if(touchingShield && yVel < HM_MIN_SPEED) {
      hm_data[2] = shieldY + SHIELD_HEIGHT/2 + HM_DIAM/2; // yPos
  }

  // -> shield is enhanced IF currentTime - shield.timeEnhanced <= SHIELD_ARMING_WINDOW

}

void update_shield_physics(int force, int xVel, int xAccl, int xPos, int touchingWall, int SHIELD_MASS, int SHIELD_BOUNCE_ENABLED, int SHIELD_LENGTH, int LEFT_WALL_X, int RIGHT_WALL_X, int * shield_data) {

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

void laser_valid(int num_lasers, int * data) {
  data[0] = 0;
  data[1] = num_lasers;
  if(num_lasers > 0) {
      data[0] = 1;
      data[1]--;
  }
}