#ifndef ROBOT_H
#define ROBOT_H

#include "../assignment.h"
#include "../tekmap/tekmap.hpp"
#include "../libraries/Image.hpp"

#include <vector>

#define MOVE_N 0
#define MOVE_NE 1
#define MOVE_E 2
#define MOVE_SE 3
#define MOVE_S 4
#define MOVE_SW 5
#define MOVE_W 6
#define MOVE_NW 7


// the robot does somehow need a pointer to the map or other way to access it
// this might need to be included in the functions as a pointer or a function able to get it from a pointer stored in its memory

// for this .h it is assumed that a position class was made which is able to represent a position in the workspace.

class robot
{
public:
	robot(shared_ptr<Image> map);

	// functionality
	/**
	 * @brief move tells the robot to go in one of the 8 possible directions (MOVE_S, MOVE_SE, ...)
	 * @param direction integer which detemines the direction
	 * @return if it was able to go there or not
	 */
	bool move(int direction);

	/**
	 * @brief pickupCup picks up a cup if it is within reach
	 * @param cupPosition the position of the cup
	 * @return if the cup could be picked up = true, if cup not picked up = false
	 */
	bool pickupCup(pos_t cupPosition);

	/**
	 * @brief pickupAllCups picks up all the cups in the vector which are within range
	 * @param cups pointer to vector of cups to be picked up
	 * @return if cupholder is full = false, else = true
	 */
	bool pickupCupsInRange(std::vector<pos_t> * cups);

	/**
	 * @brief startCupScan scans for cups in its range
	 * @param cups pointer to a vector used to hold the poistion of the cups in range
	 * @return if cups where detected, true = yes, false = no cups
	 */
	bool startCupScan();

	/**
	 * @brief emptyCupCarrier empties the cup storage on
	 * @return if cups where successfully emptied
	 */
	bool emptyCupCarrier();

	/**
	 * @brief cleanMapForCups initiates the process of cleaning the map for cups
	 */
	void cleanMapForCups();

	/**
	 * @brief cleanFloorOnMap initiates the process of cleaning the floor on the map
	 */
	void cleanFloorOnMap();

	/**
	 * @brief cleanRoom initiates the process of cleaning the floor on the current room
	 * @param doOnRun call this function on every step (like pick up cups)
	 * @param coverageWidth the rnage it is able to cover by walking around the room (scan or wash radius)
	 */
	void cleanRoom(void (*doOnCoverage)(), void (*doAfterCoverage)(), int coverageWidth);

	/**
	 * @brief cupClean run necessary funcs to clean the map for cups
	 */
	void cupClean();

	// get'ers
	/**
	 * @brief getCupsHolding getter
	 * @return number of cups the robot holds
	 */
	int getCupsHolding();

	/**
	 * @brief getRobotWidth getter
	 * @return the width of the robot, in px
	 */
	int getRobotWidth();

	/**
	 * @brief getCupSearchRadius getter
	 * @return the radius, in px, the robot is able to search for cups
	 */
	int getCupSearchRadius();

	/**
	 * @brief getCupPickRadius getter
	 * @return the radius, in px, the robot is able to pick up cups within
	 */
	int getCupPickRadius();

	/**
	 * @brief getRobotPos getter
	 * @return the position at which the robot is
	 */
	pos_t getRobotPos();

	/**
	 * @brief getDistanceWalked getter
	 * @return the distance the robot has walked till now in 8-point connectivity.
	 */
	double getDistanceWalked();

	// set'ers
	/**
	 * @brief setCups setter
	 * @param cups sets the number of cups the robot holds
	 */
	void setCups(int cups);

	/**
	 * @brief setRobotWidth setter
	 * @param width the width of the robot, in px
	 */
	void setRobotWidth(int width);

	/**
	 * @brief setCupSearchRadius setter
	 * @param searchRadius the radius the robot is able to search within, in px.
	 */
	void setCupSearchRadius(int searchRadius);

	/**
	 * @brief setCupPickRadius setter
	 * @param pickRadius the radius, in px, the robot is able to pick up cups within
	 */
	void setCupPickRadius(int pickRadius);

	/**
	 * @brief setDistanceWalked setter
	 * @param distance the distance the robot has walked till now
	 */
	void setDistanceWalked(double distance);

	/**
	 * @brief setRobotPos setter
	 * @param position the new possition of the robot
	 */
	void setRobotPos(pos_t position);

private:
	// parameters for the robots functionality
	int cupsHolding;
	int robotWidth;
	int cupSearchRadius;
	int cupPickRadius;
	double distanceWalked;
	// parameters for robots position
	pos_t robotPosition;

	// maps
	brushfire_map * mapBrush;
	wavefront_map * mapWave;
	pixelshade_map * mapNormal; // are the cups on this map type??
	pixelshade_map * mapRooms;

	// vector to hold the cups
	std::vector<pos_t> cupsToPickUp;

};


#endif // ROBOT_H
