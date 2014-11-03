/** @file */
#pragma once
#define ROBOT_START_X               2626
#define ROBOT_START_Y               1313
#define ROBOT_DYNAMICS_RADIUS       4   // 0.4 meters = 4 pixels radius
#define ROBOT_SCANNER_RADIUS        20  // 2 meters = 20 pixels radius
#define ROBOT_ARM_RADIUS            10  // 1 meter = 10 pixels radius
#define ROBOT_CUP_CAPACITY          20  // Robot can hold no more than 20 cups
#define ROBOT_SPEED_PIX_PER_S       13.8888888888888888888888888888888888889    // 125/9
//#define ROBOT_SPEED_PIX_PER_S       ( ( (long double)(125) ) / ( (long double)(9.0)) ) )


#define WSPACE_FREE                 255
#define WSPACE_IS_FREE( NUM )       ( NUM == WSPACE_FREE )
#define WSPACE_CUP                  150
#define WSPACE_IS_CUP( NUM )        ( NUM == WSPACE_CUP )
#define WSPACE_OL_STATION           100
#define WSPACE_IS_OL_STATION( NUM ) ( NUM == WSPACE_OL_STATION )
#define WSPACE_IS_ELEVATOR( NUM )   ( (NUM==128)||(NUM==129)||(NUM==130)||(NUM==131)||(NUM==132) )
/*
 * #define WSPACE_IS_OBSTACLE( NUM )   ( !( WSPACE_IS_FREE(NUM)|| \
 *      WSPACE_IS_CUP(NUM)||WSPACE_IS_OL_STATION(NUM)||WSPACE_IS_ELEVATOR(NUM) ) )
 */
#define WSPACE_IS_OBSTACLE( NUM )   (NUM==0)

#define WSPACE_DOORWAY_SIZE        7
#define WSPACE_DOOR_UNCERTAINTY    2

#define WAVE_VAL_UNV           -1
#define WAVE_VAL_GOAL          0
