/** @file */
#include "robot_old.h"
#include <cmath>


#define NORMAL 0
#define TESTROOM 1

#define RUNMODE TESTROOM


#define WALK_DOWN 0
#define WALK_UP 1
#define WALK_AROUND 2

// used for walking in a room
int walk(pos_t currentPos, shared_ptr<brushfire_map> map, int direction);


robot_old::robot_old(shared_ptr<Image> map):cupsHolding(0), mapBrush(map, pos_t(ROBOT_START_X,ROBOT_START_Y)), mapNormal(map), mapTrack(map)
{
	setCupPickRadius (ROBOT_ARM_RADIUS);
	setCupSearchRadius (ROBOT_SCANNER_RADIUS);
	setDistanceWalked (0);
	setRobotWidth (ROBOT_DYNAMICS_RADIUS);
	setRobotPos (pos_t(ROBOT_START_X,ROBOT_START_Y));

	// Create a vector of door coordinates
	doorDetector mydetective;
	vector<pos_t> The_Doors = mydetective.detect_doorways(map, mapBrush);

	// Create a map with every room sealed off.
	mapRooms = make_shared<pixelshade_map> (mydetective.door_step(map, mapBrush, The_Doors)); // only works if brush was called with coordinate!
	mapRoomBrush = make_shared<brushfire_map> ((const pixelshadeMap &) *mapRooms); // defect?
}


void robot_old::saveNormalMap(shared_ptr<Image> map, std::string name)
{
	mapTrack.shade (map);
	map->saveAsPGM (name);
}


bool robot_old::move(int direction)
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
	if(WSPACE_IS_FREE (mapNormal.coordVal (newPosition)) || WSPACE_IS_CUP(mapNormal.coordVal (newPosition)) || WSPACE_IS_OL_STATION(mapNormal.coordVal (newPosition)))
	{
		setRobotPos(newPosition);
		setDistanceWalked(walked);
		result = true;

#if RUNMODE == TESTROOM
		// track movement
		mapTrack.coordVal (newPosition) = mapTrack.coordVal (newPosition) + 40;
#endif
	}

	return result;
}


bool robot_old::pickupCup(pos_t cupPosition)
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

bool robot_old::pickupCupsInRange(std::vector<pos_t> * cups)
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


bool robot_old::startCupScan()
{
	// start cupscan through other class (pass pointer to map and cup vector?)
	bool result = false;
	// call with shared_ptr<vector <pos_t>> cups
	return result;
}


bool robot_old::emptyCupCarrier()
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


void robot_old::cleanRoom(void (*doOnCoverage)(), void (*doAfterCoverage)(), int coverageWidth)
{
	int dir;
	char maxLvl = mapRoomBrush->coordVal (getRobotPos ());
	char currentLvl = coverageWidth;
	pos_t history[2] = {pos_t(-1,-1),pos_t(-2,-2)};

	// go to edge of a room (the value of coverage range)
	while (mapBrush.coordVal (getRobotPos ()) > currentLvl)
	{
		// get walking direction
		dir = walk(getRobotPos(),mapRoomBrush, WALK_DOWN);
		// move
		move(dir);
		// execute this on evrey move (eg scan)
		doOnCoverage();
	}

	// walk around in circles and when returned to "start" walk towards center
	bool roomDone = false;
	pos_t startPos;

	while (!roomDone) {
		startPos = getRobotPos ();
		// walk around on the lvl till back on original spot
		do{
			// get walking direction
			dir = walk(getRobotPos (),mapRoomBrush, WALK_AROUND);
			// move
			move (dir);
			// execute this on evrey move (eg scan)
			doOnCoverage();
			// if looping in same area or stuck, stop
			if((history[0] == getRobotPos ()) || (history[1] == getRobotPos ()) || (history[0] == history[1]))
			{
				roomDone = true;
			}
			// create history
			history[1] = history[0];
			history[0] = getRobotPos ();

		} while (getRobotPos() != startPos && !roomDone);

		// check if room done
		if(mapRoomBrush->coordVal (getRobotPos ()) == maxLvl || roomDone)
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
			while (mapRoomBrush->coordVal (getRobotPos ()) < currentLvl && !roomDone)
			{
				dir = walk(getRobotPos (),mapRoomBrush, WALK_UP);
				if(dir == -1)
				{
					roomDone = true;
				}
				else {
					move(dir);
					doOnCoverage();
				}
			}
		}
	}

	doAfterCoverage();
}

// comp: a < b => down, a > b => up and a == b => around
int walk(pos_t currentPos, shared_ptr<brushfire_map> map, int direc)
{
	// returns direction (N/S/NW...)
	pos_t dir(0,1);
	pos_t position = pos_t(currentPos.x()-1,currentPos.y()-1);
	char last = map->coordVal (pos_t(currentPos.x(),currentPos.y()-1));
	char current = map->coordVal (currentPos);

	int result[8] = {MOVE_SW, MOVE_W, MOVE_NW, MOVE_N, MOVE_NE, MOVE_E, MOVE_SE, MOVE_S};

	// walk CW around currentpos
	for(int i = 1; i < 9; i++)
	{
		switch (direc) {
			case WALK_DOWN:
				// return the first direction that leads down
				if (map->coordVal (position) < current)
				{
					return result[(i-1)];
				}
				break;
			case WALK_UP:
				// return the first direction that leads up
				if (map->coordVal (position) > current)
				{
					return result[(i-1)];
				}
				break;
			case WALK_AROUND:
				// return the first direction that leads around (stalk the circle one closer to the wall)
				if ((map->coordVal (position) == current) && (last == (current-1)))
				{
					return result[(i-1)];
				}
				last = map->coordVal (position);
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
	return -1;
}


void robot_old::cupClean()
{
	// scan
	startCupScan();
	pickupCupsInRange(&cupsToPickUp);
}


void robot_old::cleanMapForCups()
{
	// run around map :)
}

void robot_old::cleanFloorOnMap()
{
	// run, baby run!
}

int robot_old::getCupsHolding()
{
	return cupsHolding;
}


int robot_old::getRobotWidth()
{
	return robotWidth;
}

int robot_old::getCupSearchRadius()
{
	return cupSearchRadius;
}


int robot_old::getCupPickRadius()
{
	return cupPickRadius;
}


pos_t robot_old::getRobotPos()
{
	return robotPosition;
}

double robot_old::getDistanceWalked()
{
	return distanceWalked;
}

void robot_old::setCups(int cups)
{
	if(cups <= ROBOT_CUP_CAPACITY)
	{
		cupsHolding = cups;
	}
}


void robot_old::setRobotWidth(int width)
{
	robotWidth = width;
}


void robot_old::setCupSearchRadius(int searchRadius)
{
	cupSearchRadius = searchRadius;
}


void robot_old::setCupPickRadius(int pickRadius)
{
	cupPickRadius = pickRadius;
}


void robot_old::setDistanceWalked(double distance)
{
	distanceWalked = distance;
}

void robot_old::setRobotPos(pos_t position)
{
	robotPosition = position;
}
