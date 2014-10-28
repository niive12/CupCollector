#ifndef ROBOT_H
#define ROBOT_H

#include <vector>


// for this .h it is assumed that a position class was made which is able to represent a position in the workspace.

class robot
{
public:
	robot();

	// functionality
	bool move(int direction);
	bool pickupCup(position cupPosition);
	bool startCupScan(vector <position> * cups);
	bool emptyCupCarrier();

	void cleanMapForCups();
	void cleanFloorOnMap();

	// get'ers
	int getCupsHolding();
	int getRobotWidth();
	int getCupSearchRadius();
	int getCupPickRadius();
	position getRobotPos();
	double getDistanceWalked();

	// set'ers
	void setCups(int cups);
	void setRobotWidth(int robotWidth);
	void setCupSearchRadius(int searchRadius);
	void setCupPickRadius(int pickRadius);
	void setDistanceWalked(double distance);

private:
	// parameters for the robots functionality
	int cupsHolding;
	int robotWidth;
	int cupSearchRadius;
	int cupPickRadius;
	double distanceWalked;
	// parameters for robots position
	position robotPosition;


};


#endif // ROBOT_H
