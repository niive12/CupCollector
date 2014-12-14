/**
 * @file robot.h
 */
#pragma once
#include "../tekmap/pixelshade.h"
#include "../tekmap/brushfire.h"
#include "../tekmap/wavefront.h"
#include "../doordetector/doordetector.h"
#include "../scanner/scanner.h"


using namespace std;

/**
 * @brief The robot class
 */
class robot {
protected:
	size_t r_dynamics;
	size_t cupCap;
	scanner cupscanner;
	scanner cuppicker;
	size_t number_of_cups;

	/**
	 * @brief The mapWrap class holds maps used by both cup_scan and floor_sweep.
	 *
	 * The robot holds one instance of this class (myMaps).
	 */
	class mapWrap {
	protected:
		shared_ptr<pixelshadeMap> originalMap;
		shared_ptr<pixelshadeMap> configurationSpaceMap;
		shared_ptr<pixelshadeMap> doorStepsMap;
		shared_ptr<norm2BrushfireMap> m_norm2BrushfireMap;
		shared_ptr<norm2BrushfireMap> m_norm2BrushfireDoorsMap;
		unordered_set<pos_t> freespaceSet;
	public:
		mapWrap(){}
		mapWrap(shared_ptr<Image> img):originalMap(make_shared<pixelshadeMap>(img)) {}
		mapWrap(shared_ptr<Image> img, const pos_t &reachableCoordinate):mapWrap(img) {
			freespaceSet =originalMap->findFreespace(reachableCoordinate);
			m_norm2BrushfireMap = make_shared<norm2BrushfireMap>((*originalMap));
		}
		mapWrap(shared_ptr<Image> img, const pos_t &reachableCoordinate,
				size_t robot_dynamics_radius):
			mapWrap(img,reachableCoordinate) {
			configurationSpaceMap = make_shared<pixelshadeMap>(img);
			pad<norm2BrushfireMap>((*configurationSpaceMap),(*m_norm2BrushfireMap),robot_dynamics_radius);
			for(coordIndexType x=0; x<coordIndexType(configurationSpaceMap->getWidth()); ++x)
				for(coordIndexType y=0; y<coordIndexType(configurationSpaceMap->getHeight()); ++y)
					if(freespaceSet.find(pos_t(x,y)) == freespaceSet.end())
						configurationSpaceMap->coordVal(x,y)=WSPACE_OBSTACLE;
			brushfireMap brush((*originalMap),reachableCoordinate);
			doorDetector mydetective;
			vector<pos_t> The_Doors = mydetective.detect_doorways(img, brush);
			doorStepsMap = make_shared<pixelshadeMap>(mydetective.door_step(img, brush, The_Doors));
			m_norm2BrushfireDoorsMap = make_shared<norm2BrushfireMap>((*doorStepsMap));
		}
		shared_ptr<pixelshadeMap> getOriginal() const { return originalMap; }
		shared_ptr<pixelshadeMap> getConfigurationSpace() const { return configurationSpaceMap; }
		shared_ptr<pixelshadeMap> getDoorStepsMap() const { return doorStepsMap; }
		shared_ptr<norm2BrushfireMap> getNorm2Brushfire() const { return m_norm2BrushfireMap; }
		shared_ptr<norm2BrushfireMap> getNorm2BrushfireDoors() const { return m_norm2BrushfireDoorsMap; }
		const unordered_set<pos_t> & getFreespace() const {return freespaceSet; }
	} myMaps;

	/**
	 * @brief The movement class Robot path storage class
	 * Stores the robot path (that's why it inherits list<pos_t>).
	 * Initialized with all parameters.
	 * Path can be reset to one position (a starting coordinate).
	 *
	 * The robot holds one instance of this class (myMoves).
	 */
	class movement : protected list<pos_t> {
	protected:
		double pathLength;
		double speed_kmh;
	public:
		movement():pathLength(0.0),speed_kmh(ROBOT_SPEED_PIX_PER_H) {}
		movement(double speed_kmh, pos_t start):pathLength(0.0),speed_kmh(speed_kmh) {
			push_back(start);
		}
		void clearPath(pos_t new_start) {
			pathLength=0;
			clear();
			push_back(new_start);
		}
		void move(const pos_t &to) {
			if( (((to.cx())-(back().cx()))==0) != (((to.cy())-(back().cy()))==0) )
				pathLength+=1.4142135623730950488016887242097;
			else
				pathLength+=1;
			push_back(to);
		}
		pos_t currentPosition() { return back(); }
		const pos_t& start() { return front(); }
		double path_length() { return pathLength; }
		double path_time_hours() { return pathLength/speed_kmh; }
		list<pos_t>::iterator begin() { return this->list<pos_t>::begin(); }
		list<pos_t>::iterator end() { return this->list<pos_t>::end(); }
	} myMoves;

	/**
	 * @brief getBrushEdges Finds the set of coordinates (edges) with certain brushfire values.
	 * All coordinates with
	 *		brushfire value == radius + integer multiple of diameter
	 * qualify for the set.
	 *
	 * @param freespace The freespace (that the robot can reach)
	 * @param brush The brushfireMap or norm2BrushfireMap to find edges in
	 * @param radius Radius (robot width for floor sweeper)
	 * @return Brushfire edges
	 */
	template<typename BrushmapT>
	static unordered_set<pos_t> getBrushEdges(const unordered_set<pos_t> &freespace, const BrushmapT &brush, size_t radius);

	/**
	 * @brief getClosestCoord Finds closest coordinate using normInf wavefront algorithm
	 * @param img Configuration space to find closest coordinate in
	 * @param coordinatesToSearch The found coordinate will be from this set
	 * @param currentPos The coordinate to find the closest coordinate to
	 * @return The found coordinate if one was found. currentPos otherwise.
	 */
	static pos_t getClosestCoord(const pixelshadeMap &img,
								 const unordered_set<pos_t> &coordinatesToSearch, const pos_t &currentPos);

	/**
	 * @brief pad Pads the image according to the brushfire map and a padding threshold
	 * @param imgToPad The image to pad
	 * @param brush The brushfireMap or norm2BrushfireMap
	 * @param padValue Padding threshold (anything equal to or below padValue will be padded)
	 */
	template<typename BrushmapT>
	static void pad(pixelshadeMap &imgToPad, const BrushmapT &brush, const size_t padValue);

	/**
	 * @brief getLocalMaxima Using a 3-4 point mesh in the x and the y axis, finds brushfire local maxima.
	 * @param img The image to find local maxima in.
	 * @param brush The brushfireMap or norm2BrushfireMap to find local maxima in.
	 * @param validFreespaceCoord A valid freespace coordinate (we only want local maxima in freespace)
	 * @param twoPointMaxima Boolean. True if you want to accept two-point maxima (eg 123321 accepts 3 as local maximum).
	 * @return Set of brushfire local maxima found in img, using brush.
	 *
	 * @todo Instead of generating freespace internally, pass the freespace by reference.
	 */
	template<typename BrushmapT>
	static unordered_set<pos_t> getLocalMaxima(const pixelshadeMap &img,
											   const BrushmapT &brush,
											   const pos_t &validFreespaceCoord,
											   bool twoPointMaxima=true);

	/**
	 * @brief chooseSubset Adds some of the coordinates from the set to choose from to the coordSet.
	 * Works like getRemainders, except that only coordinates from the set to choose from
	 * get added to the resulting set.
	 * @param chooseFrom The set to choose from (for example, brushfire local maxima).
	 * @param coordSet The coordinate set we want to expand to include some more (unscanned) coordinates.
	 * @param radius The radius to scan around each local maximum.
	 */
	static void chooseSubset(const unordered_set<pos_t> &chooseFrom,
								  unordered_set<pos_t> &coordSet,
								  size_t radius=ROBOT_DYNAMICS_RADIUS);

	/**
	 * @brief isRemainder Supplements getRemainders()
	 * A remainder is defined as a coordinate that can not be seen
	 * from any of the coordinates in coords, within the specified radius.
	 * @param center The freespace coordinate to scan around.
	 * @param coords The set of coordinates (the non-remainders).
	 * @param radius The radius to scan around center.
	 * @return True if no coordinate in coords was found to be within circle around center.
	 *			False otherwise.
	 */
	static bool isRemainder(pos_t center, const unordered_set<pos_t> &coords, unsigned int radius);

	/**
	 * @brief getRemainders Finds the coordinates that the other preprocessing missed.
	 * Should be called as the last preprocessing step.
	 *
	 * @param img A map of the configuration space (padded workspace)
	 * @param coordSet The coordinates that the robot already knows it must visit
	 * @param freespaceCoord A coordinate reachable by the robot
	 * @param radius The radius (the robot with for floor sweeper)
	 */
	static void getRemainders(const pixelshadeMap &img,
							  unordered_set<pos_t> &coordSet,
							  const pos_t &freespaceCoord,
							  size_t radius=ROBOT_DYNAMICS_RADIUS);

	/**
	 * @brief getFloorSweepCoordinates Generates complete set of coordinates the floor sweeper robot must visit.
	 *
	 * How it works:
	 * 1. Create an empty coordinate set, called coords.
	 * 2. Get brushfire edges in norm2BrushfireDoors, accepting only freespace coordinates.
	 * 3. Add all edges from step 2 to coords.
	 * 4. Find brushfire local maxima (in norm2BrushfireDoors), located in configuration space.
	 * 5. Add the minimum necessary amount of coordinates from step 4. to coords.
	 * 6. Scan the freespace for coordinates that are not touched by the robot if it stood
	 *		exactly on all the coordinates of coords.
	 * 7. Add the coordinates from 6. (the "remainders") to coords.
	 * 8. Return coords.
	 *
	 * @param img Image
	 * @param original Original
	 * @param freespace Freespace
	 * @param configurationSpace Configuration space
	 * @param norm2BrushfireDoors Norm2 brushfire map generated from a map with doors seen as obstacles
	 * @param reachable_coordinate A reachable coordinate (the starting coordinate, for example)
	 * @param robot_dynamics_radius The radius of the robot
	 * @param makePGMs True if you want a lot of .pgm file output. False if you don't.
	 * @return Complete set of coordinates the floor sweeper robot must visit.
	 */
	static unordered_set<pos_t> getFloorSweepCoordinates(shared_ptr<Image> img,
														 const pixelshadeMap &original,
														 const unordered_set<pos_t> &freespace,
														 const pixelshadeMap &configurationSpace,
														 const norm2BrushfireMap &norm2BrushfireDoors,
														 const pos_t &reachable_coordinate,
														 size_t robot_dynamics_radius=ROBOT_DYNAMICS_RADIUS,
														 bool makePGMs=false);

	/**
	 * @brief getCupScanCoordinates Calls getFloorSweepCoordinates(), with a different radius.
	 */
	static unordered_set<pos_t> getCupScanCoordinates(shared_ptr<Image> img,
														 const pixelshadeMap &original,
														 const unordered_set<pos_t> &freespace,
														 const pixelshadeMap &configurationSpace,
														 const norm2BrushfireMap &norm2BrushfireDoors,
														 const pos_t &reachable_coordinate,
														 size_t robot_scanner_radius=ROBOT_SCANNER_RADIUS);

	/**
	 * @brief pickupCups Picks up cups and updates variables.
	 * @param cupspace A pixelshadeMap that may be modified (when a cup is removed).
	 * @param coordSet The set of coordinates that the robot must visit to guarantee detection of all cups.
	 * @param missed_cups A set holding the cups that are not yet picked up.
	 * @param reachableCSpace All reachable coordinates in the configuration space.
	 * @param img Image (if makePGMs is true, img will have its cups removed).
	 * @param makePGMs Whether or not to remove cups from img.
	 * @param cupsCollected Records all collected cups (only for record keeping!).
	 * @param c The coordinate to scan around.
	 */
	void pickupCups(pixelshadeMap &cupspace, unordered_set<pos_t> &coordSet,
					set<pos_t> &missed_cups, const unordered_set<pos_t> &reachableCSpace,
					shared_ptr<Image> img, bool makePGMs,
					unordered_set<pos_t> &cupsCollected, const pos_t &c);

public:
	/**
	 * @brief Robot Constructor.
	 * @param img Image.
	 * @param radius_dynamics Radius of the robot floor sweeping.
	 * @param radius_arm Radius of the robot arm's reach.
	 * @param radius_scanner Radius of the laser scanner.
	 * @param cup_capacity The maximum number of cups the robot can hold.
	 * @param speed_pix_pr_h The speed of the robot in pixels per hour.
	 * @param starting_position The starting (and ending) position of the robot. Must be a freespace coordinate!
	 */
	robot(shared_ptr<Image> img,
			size_t radius_dynamics, size_t radius_arm, size_t radius_scanner,
		  size_t cup_capacity, double speed_pix_pr_h, pos_t starting_position);

	/**
	 * @brief sweep_floor Sweeps the floor
	 * @param img The original image
	 * @param makePGMs True if you want a lot of .pgm file output. False if you don't.
	 */
	void sweep_floor(shared_ptr<Image> img, bool makePGMs=false);

	/**
	 * @brief cup_scan Moves through the map, collecting all cups found
	 *
	 * 1.	Generate preprocessing maps (configuration space and other maps).
	 * 2.	Generate list C of coordinates to visit to ensure that all
	 *		of the map gets scanned (all cups will be detected).
	 * 3.	Move the robot to the nearest coordinate in C and remove it from C.
	 *	3a.	If the robot detects a cup, the nearest-to-the-cup
	 *		configuration space coordinate gets added to C.
	 *	3b.	If the robot is within arm-reach of a cup, the cup is picked up.
	 *		3ba.	If 20 cups have been picked up, move the robot to an offloading station.
	 *				On the way, add all found cups to C.
	 *				When at offloading station, deliver cups and continue.
	 * 4.	If there are still coordinates in C, go to step 3.
	 * 5.	Output traveled length in meters and travel time in hours and minutes.
	 *
	 * @param img Original image
	 * @param makePGMs True if you want a lot of pgm output. False if you don't.
	 */
	void cup_scan(shared_ptr<Image> img, bool makePGMs=false);
};
