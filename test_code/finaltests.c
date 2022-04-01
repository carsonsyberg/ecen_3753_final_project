#include <stdlib.h>
#include "ctest.h"
#include "final.h"

//----------------------------------------------------------------------------------------------------------------------------------
/// @Makefile
/// 1. type 'make' in command line for the project to be built
/// 2. type 'make remake' to rebuild your project
/// 3. type './shortestjobfirst' to run your unit tests
//----------------------------------------------------------------------------------------------------------------------------------

CTEST_DATA(data_set0) {
    int shield_touching_wall;
    int hm_touching_wall;
    int hm_touching_shield;
    int hm_x_data[3];
    int hm_y_data[3];
    int shield_data[3];
    int laser_data[2];
};

CTEST_SETUP(data_set0) {
    //               {shield_wall, hm_wall, hm_shield}
    int touching[] = {0, 0, 0};
    //           {xAccl, xVel, xPos}
    int hm_x[] = {0, 0, 0};
    //           {yAccl, yVel, yPos}
    int hm_y[] = {0, 0, 0};
    //           {accl, vel, pos}
    int shield[] = {0, 0, 0};
    //            {numLasers, laserActive}
    int laser[] = {0, 0};
    
    init(data->shield_touching_wall, 
         data->hm_touching_wall, 
         data->hm_touching_shield, 
         data->hm_x_data, 
         data->hm_y_data, 
         data->shield_data, 
         data->laser_data,
         touching,
         hm_x,
         hm_y,
         shield,
         laser);
}

// Test0: shield_touching_wall -> touching niether wall -> output -1
CTEST2(data_set0, shield_niether_wall) {
    int shieldX = 64; // center screen
    int SHIELD_LENGTH = 40; // 40 px wide
    int LEFT_WALL_X = 0; // uses full screen
    int RIGHT_WALL_X = 128; // ^^^

    data->shield_touching_wall = shield_touching_wall(shieldX, SHIELD_LENGTH, LEFT_WALL_X, RIGHT_WALL_X);
    ASSERT_EQUAL(-1, data->shield_touching_wall);
}
// Test1: shield_touching_wall -> touching left wall -> output 0
CTEST2(data_set0, shield_left_wall) {
    int shieldX = 39; // shield should have 1 px off screen left "touching wall"
    int SHIELD_LENGTH = 40; // 40 px wide
    int LEFT_WALL_X = 0; // uses full screen
    int RIGHT_WALL_X = 128; // ^^^

    data->shield_touching_wall = shield_touching_wall(shieldX, SHIELD_LENGTH, LEFT_WALL_X, RIGHT_WALL_X);
    ASSERT_EQUAL(0, data->shield_touching_wall);
}
// Test2: shield_touching_wall -> touchgin right wall -> output 1
CTEST2(data_set0, shield_right_wall) {
    int shieldX = 89; // shield should have 1 px off screen right "touching wall"
    int SHIELD_LENGTH = 40; // 40 px wide
    int LEFT_WALL_X = 0; // uses full screen
    int RIGHT_WALL_X = 128; // ^^^

    data->shield_touching_wall = shield_touching_wall(shieldX, SHIELD_LENGTH, LEFT_WALL_X, RIGHT_WALL_X);
    ASSERT_EQUAL(1, data->shield_touching_wall);
}

// Test3: hm_touching_wall ->  touching niether wall -> output -1
CTEST2(data_set0, hm_niether_wall) {
    int hmX = 64; // center screen
    int HM_DIAM = 10; // HM is 10 px in diameter
    int LEFT_WALL_X = 0; // uses full screen
    int RIGHT_WALL_X = 128; // ^^^

    data->hm_touching_wall = hm_touching_wall(hmX, HM_DIAM, LEFT_WALL_X, RIGHT_WALL_X);
    ASSERT_EQUAL(-1, data->hm_touching_wall);
}
// Test4: hm_touching_wall -> touching left wall -> output 0
CTEST2(data_set0, hm_left_wall) {
    int hmX = 9; // HM should have 1 px off screen left "touching wall"
    int HM_DIAM = 10; // HM is 10 px in diameter
    int LEFT_WALL_X = 0; // uses full screen
    int RIGHT_WALL_X = 128; // ^^^

    data->hm_touching_wall = hm_touching_wall(hmX, HM_DIAM, LEFT_WALL_X, RIGHT_WALL_X);
    ASSERT_EQUAL(0, data->hm_touching_wall);
}
// Test5: hm_touching_wall -> touching right wall -> output 1
CTEST2(data_set0, hm_right_wall) {
    int hmX = 119; // HM should have 1 px off screen right "touching wall"
    int HM_DIAM = 10; // HM is 10 px in diameter
    int LEFT_WALL_X = 0; // uses full screen
    int RIGHT_WALL_X = 128; // ^^^

    data->hm_touching_wall = hm_touching_wall(hmX, HM_DIAM, LEFT_WALL_X, RIGHT_WALL_X);
    ASSERT_EQUAL(1, data->hm_touching_wall);
}

// Test6: hm_touching_shield -> hm is touching shield -> output 1
CTEST2(data_set0, hm_touching_shield) {
    int hmX = 64; // center screen
    int hmY = 111; // 118 - 0.5*6 - 0.5*10 + 1 = 111  places HM w/ 1 pixel inside the shield "touching"
    int shieldX = 64; // center screen
    int shieldY = 118; // places HM w/ 1 pixel inside shield (near bottom of screen)
    int SHIELD_HEIGHT = 6;
    int SHIELD_LENGTH = 40;
    int HM_DIAM = 10;

    data->hm_touching_shield = hm_touching_shield(hmX, hmY, shieldX, shieldY, SHIELD_HEIGHT, SHIELD_LENGTH, HM_DIAM);
    ASSERT_EQUAL(1, data->hm_touching_shield);
}
// Test7: hm_touching_shield -> hm is not touching shield -> output 0
CTEST2(data_set0, hm_not_touching_shield) {
    int hmX = 64; // center screen
    int hmY = 0; // top of screen
    int shieldX = 64; // center screen
    int shieldY = 118; // near bottom of screen
    int SHIELD_HEIGHT = 6;
    int SHIELD_LENGTH = 40;
    int HM_DIAM = 10;

    data->hm_touching_shield = hm_touching_shield(hmX, hmY, shieldX, shieldY, SHIELD_HEIGHT, SHIELD_LENGTH, HM_DIAM);
    ASSERT_EQUAL(0, data->hm_touching_shield);
}

// Test8: laser_valid -> have 0 lasers left -> output 0 for laser_data[0] and 0 for laser_data[1]
CTEST2(data_set0, no_lasers_left) {
    int num_lasers = 0;

    laser_valid(num_lasers, data->laser_data);
    ASSERT_EQUAL(0, data->laser_data[0]); // lasers active
    ASSERT_EQUAL(0, data->laser_data[1]); // num_lasers remaining
}
// Test9: laser_valid -> have 1 laser left -> output 1 for laser_data[0] and 0 for laser_data[1]
CTEST2(data_set0, one_laser_left) {
    int num_lasers = 1;

    laser_valid(num_lasers, data->laser_data);
    ASSERT_EQUAL(1, data->laser_data[0]); // lasers active
    ASSERT_EQUAL(0, data->laser_data[1]); // num_lasers remaining
}

// Test9: laser_valid -> have 10 lasers left -> output 1 for laser_data[0] and 9 for laser_data[1]
CTEST2(data_set0, ten_lasers_left) {
    int num_lasers = 10;

    laser_valid(num_lasers, data->laser_data);
    ASSERT_EQUAL(1, data->laser_data[0]); // lasers active
    ASSERT_EQUAL(9, data->laser_data[1]); // num_lasers remaining
}