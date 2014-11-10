/** @file */
#include "robot.h"
#include <cmath>


#define NORMAL 0
#define TESTROOM 1

#define RUNMODE TESTROOM


#define WALK_DOWN 0
#define WALK_UP 1
#define WALK_AROUND 2

// used for walking in a room
int walk(pos_t currentPos, shared_ptr<brushfire_map> map, int direction);


robot::robot(shared_ptr<Image> map):cupsHolding(0), mapBrush(map), mapNormal(map)
{
	setCupPickRadius (ROBOT_ARM_RADIUS);
	setCupSearchRadius (ROBOT_SCANNER_RADIUS);
	setDistanceWalked (0);
	setRobotWidth (ROBOT_DYNAMICS_RADIUS);

	//init the maps
	// rooms
	// Create a vector of door coordinates
	//doorDetector mydetective;
	//vector<pos_t> The_Doors = mydetective.detect_doorways(map, mapBrush);

	// Create a map with every room sealed off.
	//mapRooms = mydetective.door_step(map, mapBrush, The_Doors);

	//mapRooms = make_shared<pixelshade_map>(map);
	//mapRoomBrush = make_shared<brushfireMap>(map);

	// Create a vector of door coordinates
	doorDetector mydetective;
	vector<pos_t> The_Doors = mydetective.detect_doorways(map, mapBrush);

	// Create a map with every room sealed off.
	mapRooms = make_shared<pixelshade_map> (mydetective.door_step(map, mapBrush, The_Doors));
	mapRoomBrush = make_shared<brushfire_map> ((const pixelshadeMap &) mapRooms);

	mapBrush.shade (map);
	map->saveAsPGM("brush_test.pgm");

	mapNormal.shade (map);
	map->saveAsPGM("normal_test.pgm");

	mapRooms->shade (map);
	map->saveAsPGM("rooms_test.pgm");

	mapRoomBrush->shade (map);
	map->saveAsPGM("roombrush_test.pgm");
}

bool robot::move(int direction)
{
	pos_t newPosition = getRobotPos();
	double walked = getDistanceWalked();
	bool result = false;
	// determine new value
	switch (direction) {
		case MOVE_N:
			newPosition = pos_t(newPosition.x(), newPosition.y() + 1);
			walked += 1;
			break;
		case MOVE_NE:
			newPosition = pos_t(newPosition.x() + 1, newPosition.y() + 1);
			walked += SQRT_2;
			break;
		case MOVE_E:
			newPosition = pos_t(newPosition.x() + 1, newPosition.y());
			walked += 1;
			break;
		case MOVE_SE:
			newPosition = pos_t(newPosition.x() + 1, newPosition.y() - 1);
			walked += SQRT_2;
			break;
		case MOVE_S:
			newPosition = pos_t(newPosition.x(), newPosition.y() - 1);
			walked += 1;
			break;
		case MOVE_SW:
			newPosition = pos_t(newPosition.x() - 1, newPosition.y() - 1);
			walked += SQRT_2;
			break;
		case MOVE_W:
			newPosition = pos_t(newPosition.x() - 1, newPosition.y());
			walked += 1;
			break;
		case MOVE_NW:
			newPosition = pos_t(newPosition.x() - 1, newPosition.y() + 1);
			walked += SQRT_2;
			break;
		default:
			break;
	}

	// if newPosition is valid
	if(WSPACE_IS_FREE (mapNormal.coordVal (newPosition)))
	{
		setRobotPos(newPosition);
		setDistanceWalked(walked);
		result = true;

#if RUNMODE == TESTROOM
		// track movement
		mapNormal.coordVal (newPosition) = 200;
#endif
	}

	return result;
}


bool robot::pickupCup(pos_t cupPosition)
{
	double cupDistanceSquared;
	pos_t robotPos = getRobotPos();
	bool result = false;

	// get distance
	cupDistanceSquared = pow((robotPos.x() + cupPosition.x()),2) + pow((robotPos.y() + cupPosition.y()),2);

	// if within reach
	if(cupDistanceSquared <= pow(getCupPickRadius(),2))
	{
		if(getCupsHolding() < ROBOT_CUP_CAPACITY)
		{
			result = true;
			setCups(getCupsHolding() + 1);
			// remove cup from map
			mapNormal.coordVal (cupPosition) = WSPACE_FREE;
		}
	}
	return result;
}

bool robot::pickupCupsInRange(std::vector<pos_t> * cups)
{
	// loop through the vector to remove all the cups within range, without moving
	// return false if cup holder runs full
	bool result = true;

	// pick up cups in range
	for(size_t i = 0; i < cups->size(); i++)
	{
		if(pickupCup(cups->at (i)))
		{
			// remove cup from vector
			cups->erase(cups->begin() + i);
		}
	}

	// check if has no space left
	if(getCupsHolding() == ROBOT_CUP_CAPACITY)
	{
		result = false;
	}

	return result;
}


bool robot::startCupScan()
{
	// start cupscan through other class (pass pointer to map and cup vector?)
	bool result = false;
	// call with shared_ptr<vector <pos_t>> cups
	return result;
}


bool robot::emptyCupCarrier()
{
	bool result = false;

	// if standing on valid spot
	if(WSPACE_IS_OL_STATION(mapNormal.coordVal (getRobotPos ())))
	{
		result = true;
		setCups(0);
	}
	return result;
}


void robot::cleanRoom(void (*doOnCoverage)(), void (*doAfterCoverage)(), int coverageWidth)
{
	int dir;
	int maxLvl = mapRoomBrush->coordVal (getRobotPos ());
	int currentLvl = coverageWidth;
	pos_t stepHistory[2];
	// first generate brush of room assuming you are in the room to clean

	//generate brush

	// go to edge of a room (the value of coverage range)
	while (mapBrush.coordVal (getRobotPos ()) > currentLvl)
	{
		dir = walk(getRobotPos(),mapRoomBrush, WALK_DOWN);
		move(dir);

		doOnCoverage();
	}

	// walk around in circles and when returned to "start" walk towards center
	bool roomDone = false;
	pos_t startPos;

	while (!roomDone) {
		startPos = getRobotPos ();

		// walk around on the lvl till back on original spot
		do{
			dir = walk(getRobotPos (),mapRoomBrush, WALK_AROUND);
			move (dir);
			doOnCoverage();
		} while (getRobotPos () != startPos);

		// check if room done
		if(mapRoomBrush->coordVal (getRobotPos ()) == maxLvl)
		{
			roomDone = true;
		}
		else
		{
			// calc next lvl
			currentLvl += 2*(coverageWidth);
			if(currentLvl > maxLvl)
			{
				currentLvl = maxLvl;

			}

			// move up to next lvl
			while (mapRoomBrush->coordVal (getRobotPos ()) < currentLvl)
			{
				dir = walk(getRobotPos (),mapRoomBrush, WALK_UP);
				move(dir);
				doOnCoverage();
			}
		}
	}

	doAfterCoverage();
}

// comp: a < b => down, a > b => up and a == b => around
int walk(pos_t currentPos, shared_ptr<brushfire_map> map, int direc)
{
	// returns direction (N/S/NW...)
	pos_t dir(1,0);
	pos_t position = currentPos;
	position.x() -= 1;
	position.y() -= 1;

	int result[8] = {MOVE_SW, MOVE_W, MOVE_NW, MOVE_N, MOVE_NE, MOVE_E, MOVE_SE, MOVE_S};

	// walk CW around currentpos
	for(int i = 1; i < 9; i++)
	{
		switch (direc) {
			case WALK_DOWN:
				// return the first direction that leads down
				if (map->coordVal (position) < map->coordVal (currentPos))
				{
					return result[(i-1)];
				}
				break;
			case WALK_UP:
				// return the first direction that leads up
				if (map->coordVal (position) > map->coordVal (currentPos))
				{
					return result[(i-1)];
				}
				break;
			case WALK_AROUND:
				// return the first direction that leads around
				if (map->coordVal (position) == map->coordVal (currentPos))
				{
					return result[(i-1)];
				}
				break;
			default:
				break;
		}

		position = position + dir;

		// turn right at every corner
		if(i % 2 == 0)
		{
			pos_t temp(dir.y(),dir.x());
			dir = temp;
		}

		// invert dir every 4 steps
		if(i % 4 == 0)
		{
			dir.x() = -(dir.x());
			dir.y() = -(dir.y());
		}
	}
	// only to surpress warning :D there will always be a way down on a brush unless you are next to the wall!
	return false;
}


void robot::cupClean()
{
	// scan
	startCupScan();
	pickupCupsInRange(&cupsToPickUp);
}


void robot::cleanMapForCups()
{
	// run around map :)
}

void robot::cleanFloorOnMap()
{
	// run, baby run!
}

int robot::getCupsHolding()
{
	return cupsHolding;
}


int robot::getRobotWidth()
{
	return robotWidth;
}

int robot::getCupSearchRadius()
{
	return cupSearchRadius;
}


int robot::getCupPickRadius()
{
	return cupPickRadius;
}


pos_t robot::getRobotPos()
{
	return robotPosition;
}

double robot::getDistanceWalked()
{
	return distanceWalked;
}

void robot::setCups(int cups)
{
	if(cups <= ROBOT_CUP_CAPACITY)
	{
		cupsHolding = cups;
	}
}


void robot::setRobotWidth(int width)
{
	robotWidth = width;
}


void robot::setCupSearchRadius(int searchRadius)
{
	cupSearchRadius = searchRadius;
}


void robot::setCupPickRadius(int pickRadius)
{
	cupPickRadius = pickRadius;
}


void robot::setDistanceWalked(double distance)
{
	distanceWalked = distance;
}

void robot::setRobotPos(pos_t position)
{
	robotPosition = position;
}
