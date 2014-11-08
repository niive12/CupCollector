//#include "robot.h"

//robot::robot(shared_ptr<Image> map)
//{
//	setCupPickRadius (ROBOT_ARM_RADIUS);
//	setCupSearchRadius (ROBOT_SCANNER_RADIUS);
//	setDistanceWalked (0);
//	setRobotWidth (ROBOT_DYNAMICS_RADIUS);

//	//init the maps
//	mapBrush = new brushfire_map(map);
//	mapWave = new wavefront_map(map);
//	mapNormal = new pixelshade_map(map);
//	mapRooms = new pixelshade_map(map);
//}

//bool robot::move(int direction)
//{
//	pos_t newPosition = getRobotPos();
//	double walked = getDistanceWalked();
//	bool result = false;
//	// determine new value
//	switch (direction) {
//		case MOVE_N:
//			newPosition = pos_t(newPosition.x(), newPosition.y() + 1);
//			walked += 1;
//			break;
//		case MOVE_NE:
//			newPosition = pos_t(newPosition.x() + 1, newPosition.y() + 1);
//			walked += sqrt(2);
//			break;
//		case MOVE_E:
//			newPosition = pos_t(newPosition.x() + 1, newPosition.y());
//			walked += 1;
//			break;
//		case MOVE_SE:
//			newPosition = pos_t(newPosition.x() + 1, newPosition.y() - 1);
//			walked += sqrt(2);
//			break;
//		case MOVE_S:
//			newPosition = pos_t(newPosition.x(), newPosition.y() - 1);
//			walked += 1;
//			break;
//		case MOVE_SW:
//			newPosition = pos_t(newPosition.x() - 1, newPosition.y() - 1);
//			walked += sqrt(2);
//			break;
//		case MOVE_W:
//			newPosition = pos_t(newPosition.x() - 1, newPosition.y());
//			walked += 1;
//			break;
//		case MOVE_NW:
//			newPosition = pos_t(newPosition.x() - 1, newPosition.y() + 1);
//			walked += sqrt(2);
//			break;
//		default:
//			break;
//	}

//	// if newPosition is valid
//	if(WSPACE_IS_FREE (mapNormal->coordVal (newPosition)))
//	{
//		setRobotPos(newPosition);
//		setDistanceWalked(walked);
//		result = true;
//	}

//	return result;
//}


//bool robot::pickupCup(pos_t cupPosition)
//{
//	double cupDistanceSquared;
//	pos_t robotPos = getRobotPos();
//	bool result = false;

//	// get distance
//	cupDistanceSquared = pow((robotPos.x() + cupPosition.x()),2) + pow((robotPos.y() + cupPosition.y()),2);

//	// if within reach
//	if(cupDistanceSquared <= pow(getCupPickRadius(),2))
//	{
//		if(getCupsHolding() < ROBOT_CUP_CAPACITY)
//		{
//			result = true;
//			setCups(getCupsHolding() + 1);
//			// remove cup from map
//			mapNormal->coordVal (cupPosition) = WSPACE_FREE;
//		}
//	}
//	return result;
//}

//bool robot::pickupCupsInRange(shared_ptr<vector <pos_t>> cups)
//{
//	// loop through the vector to remove all the cups within range, without moving
//	// return false if cup holder runs full
//	bool result = true;

//	// pick up cups in range
//	for(int i = 0; i < cups->size(); i++)
//	{
//		if(pickupCup(cups->at (i)))
//		{
//			// remove cup from vector
//			cups->erase(cups->begin() + i);
//		}
//	}

//	// check if has no space left
//	if(getCupsHolding() == ROBOT_CUP_CAPACITY)
//	{
//		result = false;
//	}

//	return result;
//}


//bool robot::startCupScan()
//{
//	// start cupscan through other class (pass pointer to map and cup vector?)
//	bool result = false;
//	// call with shared_ptr<vector <pos_t>> cups
//	return result;
//}


//bool robot::emptyCupCarrier()
//{
//	bool result = false;

//	// if standing on valid spot
//	if(WSPACE_IS_OL_STATION(mapNormal->coordVal (getRobotPos ())))
//	{
//		result = true;
//		setCups(0);
//	}
//	return result;
//}


//void robot::cleanRoom(void * doOnRun, int coverageWidth)
//{
//	// first generate brush of room assuming you are in the room to clean

//	//generate brush

//	// go to edge of a room (the value of coverage range)




//	if(doOnRun != nullptr)
//	{
//		doOnRun();
//	}


//}

////pos_t


//void robot::cupClean()
//{
//	// scan
//	startCupScan();
//	pickupCupsInRange(&cupsToPickUp);
//}


//void robot::cleanMapForCups()
//{
//	// run around map :)
//}

//void robot::cleanFloorOnMap()
//{
//	// run, baby run!
//}

//int robot::getCupsHolding()
//{
//	return cupsHolding;
//}


//int robot::getRobotWidth()
//{
//	return robotWidth;
//}

//int robot::getCupSearchRadius()
//{
//	return cupSearchRadius;
//}


//int robot::getCupPickRadius()
//{
//	return cupPickRadius;
//}


//pos_t robot::getRobotPos()
//{
//	return robotPosition;
//}

//double robot::getDistanceWalked()
//{
//	return distanceWalked;
//}

//void robot::setCups(int cups)
//{
//	if(cups <= ROBOT_CUP_CAPACITY)
//	{
//		cupsHolding = cups;
//	}
//}


//void robot::setRobotWidth(int width)
//{
//	robotWidth = width;
//}


//void robot::setCupSearchRadius(int searchRadius)
//{
//	cupSearchRadius = searchRadius;
//}


//void robot::setCupPickRadius(int pickRadius)
//{
//	cupPickRadius = pickRadius;
//}


//void robot::setDistanceWalked(double distance)
//{
//	distanceWalked = distance;
//}

//void robot::setRobotPos(pos_t position)
//{
//	robotPosition = position;
//}
