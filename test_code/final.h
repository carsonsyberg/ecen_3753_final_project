
#ifndef __FINAL__
#define __FINAL__

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

enum touching_wall {LEFT_WALL = 0, RIGHT_WALL = 1, NO_WALL = -1};

void init(
  int shield_touching_wall, int hm_touching_wall, int hm_touching_shield, 
  int hm_x_data[3], int hm_y_data[3], int shield_data[3], int laser_data[2],
  int touching[3], int hm_x[3], int hm_y[3], int shield[3], int laser[2]);

int shield_touching_wall(int shieldX, int SHIELD_LENGTH, int LEFT_WALL_X, int RIGHT_WALL_X);

int hm_touching_wall(int hmX, int HM_DIAM, int LEFT_WALL_X, int RIGHT_WALL_X);

int hm_touching_shield(int hmX, int hmY, int shieldX, int shieldY, int SHIELD_HEIGHT, int SHIELD_LENGTH, int HM_DIAM);

void update_hm_x_physics(int xForce, int xAccl, int xVel, int xPos, int touchingWall, int HM_MASS, int HM_DIAM, int LEFT_WALL_X, int RIGHT_WALL_X, int * hm_data);

void update_hm_y_physics(int yForce, int yAccl, int yVel, int yPos, int shieldY, int touchingShield, int shield_enhanced, int HM_MASS, int HM_DIAM, int HM_MIN_SPEED, int SHIELD_HEIGHT, int SHIELD_ENERGY_INCREASE, int SHIELD_ENERGY_REDUCTION, int * hm_data);

void update_shield_physics(int force, int xVel, int xAccl, int xPos, int touchingWall, int SHIELD_MASS, int SHIELD_BOUNCE_ENABLED, int SHIELD_LENGTH, int LEFT_WALL_X, int RIGHT_WALL_X, int * shield_data);

void laser_valid(int num_lasers, int * data);

#endif // __FINAL__