/**
 * @file test.hpp
 * @author Mikael Westermann
 */
#pragma once
#include "tekmap/tekmap.hpp"
#include <cmath>
#include <algorithm> //merge
#include <iomanip>
#include <sstream>
#include <forward_list>
#include "doordetector/doordetector.h"
#include "scanner/scanner.h"


using namespace std;

class norm2BrushfireMap;
class mapWrap;
class tester;
class simpleRobot;

/**
 * @brief The simpleRobot class Robot Path and Parameters storage class
 * Stores the robot path (that's why it inherits list<pos_t>)
 * and the robot parameters.
 * Initialized with all parameters.
 *
 * Path can be reset to one position (a starting coordinate).
 */
class simpleRobot : protected list<pos_t> {
protected:
	double pathLength;
	double speed_kmh;
	size_t r_dynamics;
	size_t r_arm;
	size_t r_scanner;
	size_t cupCap;
public:
	simpleRobot(size_t dynamics_radius, size_t arm_radius, size_t scanner_radius, size_t cup_capacity, double speed_kmh, pos_t start)
		:pathLength(0.0),speed_kmh(speed_kmh),r_dynamics(dynamics_radius),r_arm(arm_radius),r_scanner(scanner_radius),cupCap(cup_capacity)
	{
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
	size_t dynamics_radius() { return r_dynamics; }
	size_t arm_radius() { return r_arm; }
	size_t scanner_radius() { return r_scanner; }
	size_t cup_capacity() { return cupCap; }
	double path_time_hours() { return pathLength/speed_kmh; }

	list<pos_t>::iterator begin() { return this->list<pos_t>::begin(); }
	list<pos_t>::iterator end() { return this->list<pos_t>::end(); }
};

/**
 * @brief The norm2BrushfireMap class Brushfire map using norm2 potential function.
 */
class norm2BrushfireMap : public tekMap<double> {
protected:
	/**
	 * @brief findObstacleBorders Finds all obstacle borders
	 * @param img Image to find obstacle borders in
	 * @return List of borders. No duplicates (MW-guaranteed)
	 */
	virtual list< pos_t > findObstacleBorders(const pixelshadeMap &img) const
	{
		unordered_set<pos_t> resulting_coords;
		const array<array<int,2>,8> neighbours =
		{{ {-1,0}, /* W */ {1,0}, /* E */
		   {0,-1}, /* N */ {0,1}, /* S */
		   {-1,-1},/* NW */{1,-1}, /* NE */
		   {1,1}, /* SE */{-1,1} /* SW */
		 }};
		for( int x = 0; x < (int)(img.getWidth()); ++x) {
			for( int y = 0; y < (int)(img.getHeight()); ++y) {
				if( WSPACE_IS_OBSTACLE( img.const_coordVal(x,y) ) ) {
					for(auto n : neighbours) {
						pos_t w = pos_t(x+n.at(0),y+n.at(1));
						//If neighbour is within image borders:
						if( img.isInMap(w) ) {
							//if the neighbour is not an obstacle:
							if( !WSPACE_IS_OBSTACLE( img.const_coordVal(w) ) ) {
								resulting_coords.emplace(x,y);
								break;
							}
						}
					}
				}
			}
		}
		return move(list<pos_t>(resulting_coords.begin(),resulting_coords.end()));
	}

	/**
	 * @brief wave Norm2 Dijkstra's algorithm propagation
	 * @param img Image to propagate wave in
	 * @param goals The goals to propagate from (the obstacle borders!)
	 */
	virtual void wave(const pixelshadeMap &img, const list<pos_t> &goals)
	{
			const double sqrt2 =1.4142135623730950488016887242097;
			const array<array<int,2>,8> neighbours =
			{{ {-1,0}, /* W */ {1,0}, /* E */
			   {0,-1}, /* N */ {0,1}, /* S */
			   {-1,-1},/* NW */{1,-1}, /* NE */
			   {1,1}, /* SE */{-1,1} /* SW */
			 }};
			myMap.clear();
			//4000 looks nicer when printed, but any high value (INF) works:
			myMap.resize( img.getWidth(), vector<myValType>( img.getHeight() , 4000));
			//myMp.resize( img->getWidth(), vector<myValType>( img->getHeight() , numeric_limits<myValType>::max()));
			vector<vector<bool> > visited(img.getWidth(),vector<bool>(img.getHeight(),false));
			priority_queue<edgeType,deque<edgeType>,edgeComp > q;
			for(auto i : goals) {
					(myMap[i.cx()])[i.cy()] = WAVE_VAL_GOAL;
					q.emplace(i,(myMap[i.cx()])[i.cy()]);
			}
			while(!q.empty()) {
					edgeType cur = q.top();
					q.pop();
					visited[cur.first.cx()][cur.first.cy()]=true;
					for(auto n : neighbours) {
							pos_t w = cur.first+pos_t(n.at(0),n.at(1));
							//If neighbour is within image borders:
							if( img.isInMap(w) ) {
									//if the neighbour is an unvisited non-obstacle:
									if( ( !WSPACE_IS_OBSTACLE( img.const_coordVal(w) ) )
													&& ( !( (visited[w.cx()])[w.cy()] ) )
													&& ((myMap[w.cx()])[w.cy()] != WAVE_VAL_GOAL)
													) {
											if(
															((myMap[cur.first.cx()])[cur.first.cy()]
															 +(((n.at(0)==0) || (n.at(1)==0))?1.0:sqrt2))
															<((myMap[w.cx()])[w.cy()])
															) {
													//increment:
													(myMap[w.cx()])[w.cy()]
																	= (myMap[cur.first.cx()])[cur.first.cy()]
																	+(((n.at(0)==0) || (n.at(1)==0))?1.0:sqrt2);
													//Add it to the wavefront
													q.emplace(w,(myMap[w.cx()])[w.cy()]);
											}
									}
							}
					}
			}
	}

public:
	/**
	 * @brief norm2BrushfireMap Constructor. Just pass a pixelshadeMap.
	 * @param img A pixelshadeMap.
	 */
	norm2BrushfireMap(const pixelshadeMap &img) {
		list<pos_t> borders = findObstacleBorders(img);
		wave(img,borders);
	}

};

/**
 * @brief The tester class Should be rewritten.
 * Although this class should be rewritten,
 * it is thoroughly documented, because, why not.
 * It's an experimental class designed for solving the
 * entire assignment.
 */
class tester {
protected:
	class mapWrap {
	protected:
		shared_ptr<pixelshadeMap> originalMap;
		shared_ptr<pixelshadeMap> configurationSpaceMap;
		shared_ptr<pixelshadeMap> doorStepsMap;
		shared_ptr<norm2BrushfireMap> m_norm2BrushfireMap;
		shared_ptr<norm2BrushfireMap> m_norm2BrushfireDoorsMap;


		unordered_set<pos_t> freespaceSet;
	public:
		mapWrap(shared_ptr<Image> img):
			originalMap(make_shared<pixelshadeMap>(img))
		{ }

		mapWrap(shared_ptr<Image> img, const pos_t &reachableCoordinate):
			mapWrap(img)
		{
			freespaceSet =originalMap->findFreespace(reachableCoordinate);
			m_norm2BrushfireMap = make_shared<norm2BrushfireMap>((*originalMap));
		}

		mapWrap(shared_ptr<Image> img, const pos_t &reachableCoordinate,
				size_t robot_dynamics_radius):
			mapWrap(img,reachableCoordinate)
		{
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
	};



public:
	template<typename BrushmapT>
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
	static unordered_set<pos_t> getBrushEdges(const unordered_set<pos_t> &freespace, const BrushmapT &brush, size_t radius)
	{
		unordered_set<pos_t> result;
		for(coordIndexType x = 0; x<(coordIndexType)brush.getWidth(); ++x) {
			for(coordIndexType y = 0; y<(coordIndexType)brush.getHeight(); ++y) {
				if(freespace.find(pos_t(x,y))!=freespace.end())
					if((((size_t)(brush.const_coordVal(x,y)-1))%(2*radius))==radius) //round?
					{
						result.emplace(x,y);
					}
			}
		}
		return move(result);
	}

	/**
	 * @brief getClosestCoord Finds closest coordinate using normInf wavefront algorithm
	 * @param img Configuration space to find closest coordinate in
	 * @param coordinatesToSearch The found coordinate will be from this set
	 * @param currentPos The coordinate to find the closest coordinate to
	 * @return The found coordinate if one was found. currentPos otherwise.
	 */
	static pos_t getClosestCoord(const pixelshadeMap &img,
								 const unordered_set<pos_t> &coordinatesToSearch, const pos_t &currentPos)
	{
		pos_t result=currentPos;
		const array<array<int,2>,8> neighbours =
		{{ {-1,0}, /* W */ {1,0}, /* E */
		   {0,-1}, /* N */ {0,1}, /* S */
		   {-1,-1},/* NW */{1,-1}, /* NE */
		   {1,1}, /* SE */{-1,1} /* SW */
		 }} ;
		unordered_set<pos_t> visited;
		visited.insert(currentPos);
		queue<pos_t> q;
		q.push(currentPos);
		while((!q.empty())&&(result==currentPos)) {
			pos_t v = q.front();
			q.pop();
			for(auto n : neighbours) {
				pos_t w = v+pos_t(n.at(0),n.at(1));
				if( (visited.insert(w).second)
						&& (!(WSPACE_IS_OBSTACLE(img.const_coordVal(w)))) )
				{
					if( (coordinatesToSearch.find(w)!=coordinatesToSearch.end()) ) {
						result=w;
						break;
					}
					else
						q.push(w);
				}
			}
		}
		return move(result);
	}

	template<typename BrushmapT>
	/**
	 * @brief pad Pads the image according to the brushfire map and a padding threshold
	 * @param imgToPad The image to pad
	 * @param brush The brushfireMap or norm2BrushfireMap
	 * @param padValue Padding threshold (anything equal to or below padValue will be padded)
	 */
	static void pad(pixelshadeMap &imgToPad, const BrushmapT &brush, const size_t padValue) {
		for(coordIndexType x=0; x<(coordIndexType)imgToPad.getWidth(); ++x)
			for(coordIndexType y=0; y<(coordIndexType)imgToPad.getHeight(); ++y)
				if(brush.const_coordVal(x,y) <= (typename BrushmapT::myValType)(padValue))
					imgToPad.coordVal(x,y) = WSPACE_OBSTACLE;
	}

	template<typename BrushmapT>
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
	static unordered_set<pos_t> getLocalMaxima(const pixelshadeMap &img,
											   const BrushmapT &brush,
											   const pos_t &validFreespaceCoord,
											   bool twoPointMaxima=true)
	{
		unordered_set<pos_t> result;
		unordered_set<pos_t> freespace = img.findFreespace(validFreespaceCoord);
		for(coordIndexType x=0;x<(coordIndexType)(img.getWidth());++x) {
			coordIndexType y0=2, y1=1, y2=0;
			while(y0<(coordIndexType)(img.getHeight())) {
				if(		((brush.const_coordVal(x,y0))<(brush.const_coordVal(x,y1)))
						&&(twoPointMaxima?
							((brush.const_coordVal(x,y2))<=(brush.const_coordVal(x,y1)))
						:	((brush.const_coordVal(x,y2))<(brush.const_coordVal(x,y1))) )
						) {
					pos_t pos(x,y1);
					if(freespace.find(pos)!=freespace.end()) {
						if( (( std::max((brush.const_coordVal(x,y2)),(brush.const_coordVal(x,y1)))
							  -std::min((brush.const_coordVal(x,y2)),(brush.const_coordVal(x,y1))))
							  <1) ) {
							if( ( (std::max((brush.const_coordVal(x,y2-1)),(brush.const_coordVal(x,y0)))
								 -std::min((brush.const_coordVal(x,y2-1)),(brush.const_coordVal(x,y0))))
								  <1 ) )
								result.insert(pos);
						}
						else if( ((std::max((brush.const_coordVal(x,y2)),(brush.const_coordVal(x,y0)))
								-std::min((brush.const_coordVal(x,y2)),(brush.const_coordVal(x,y0)))))
								 <1) {
							result.insert(pos);
						}
					}
				}
				++y0;
				++y1;
				++y2;
			}
		}
		for(coordIndexType y=0;y<(coordIndexType)(img.getHeight());++y) {
			coordIndexType x0=2, x1=1, x2=0;
			while(x0<(coordIndexType)(img.getWidth())) {
				if(		((brush.const_coordVal(x0,y))<(brush.const_coordVal(x1,y)))
						&&(twoPointMaxima?
							((brush.const_coordVal(x2,y))<=(brush.const_coordVal(x1,y)))
						:	((brush.const_coordVal(x2,y))<(brush.const_coordVal(x1,y))) )
						) {
					pos_t pos(x1,y);
					if(freespace.find(pos)!=freespace.end()) {
						if( (( std::max((brush.const_coordVal(x2,y)),(brush.const_coordVal(x1,y)))
							   -std::min((brush.const_coordVal(x2,y)),(brush.const_coordVal(x1,y))))
							 <1) ) {
							if( ( (std::max((brush.const_coordVal(x2-1,y)),(brush.const_coordVal(x0,y)))
								   -std::min((brush.const_coordVal(x2-1,y)),(brush.const_coordVal(x0,y))))
								  <1 ) )
								result.insert(pos);
						}
						else if( ((std::max((brush.const_coordVal(x2,y)),(brush.const_coordVal(x0,y)))
								   -std::min((brush.const_coordVal(x2,y)),(brush.const_coordVal(x0,y)))))
								 <1) {
							result.insert(pos);
						}
					}
				}
				++x0;
				++x1;
				++x2;
			}
		}
		return move(result);
	}

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
								  size_t radius=ROBOT_DYNAMICS_RADIUS) {
		for(auto c:chooseFrom)
			if(isRemainder(c,coordSet,radius))
				coordSet.insert(c);
	}

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
	static bool isRemainder(pos_t center, const unordered_set<pos_t> &coords, unsigned int radius)
	{
		static unsigned int r=0;
		static forward_list<pos_t> circle;
		bool remainder=true;
		if(r!=radius) {
			circle.clear();
			const array<array<int,2>,8> neighbours =
			{{  {-1,0}, /* W */ {1,0},  /* E */
				{0,-1}, /* N */ {0,1},  /* S */
				{-1,-1},/* NW */{1,-1}, /* NE */
				{1,1},  /* SE */{-1,1}  /* SW */
			 }};
			r=radius;
			int rsquared = r*r;
			queue<pos_t> q;
			q.emplace(0,0);
			unordered_set<pos_t> visited;
			while(!q.empty()) {
				pos_t cur = q.front();
				q.pop();
				circle.push_front(cur);
				for(auto n:neighbours)
					if( ((cur.cx()+n[0])*(cur.cx()+n[0])
						 +(cur.cy()+n[1])*(cur.cy()+n[1]))<=rsquared )
						if((visited.emplace(cur.cx()+n[0], cur.cy()+n[1])).second)
							q.emplace(cur.cx()+n[0], cur.cy()+n[1]);
			}
		}
		for(auto i:circle) {
			try {
				if(coords.find(center+i)!=coords.end()) {
					remainder=false;
					break;
				}
			} catch (const std::out_of_range& oor) {(void)oor;}
		}
		return remainder;
	}

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
							  size_t radius=ROBOT_DYNAMICS_RADIUS) {
		unordered_set<pos_t> freespace = img.findFreespace(freespaceCoord);
		for(auto c:freespace)
			if(isRemainder(c,coordSet,radius))
				coordSet.insert(c);
	}


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
														 bool makePGMs=false)
	{
		//Go through the brushfire map and generate a set, coords, of all the coordinates
		// with brushfire values equal to certain multiples of robot
		// diameter/radius combinations.
		// This set will be called the brushfire "edges".
		unordered_set<pos_t> coords = getBrushEdges(freespace,norm2BrushfireDoors,robot_dynamics_radius);
		if(makePGMs) {
			cout << "Saving brushfire edges as \"floor_sweep_brushfire_edges.pgm\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("floor_sweep_brushfire_edges.pgm");
			original.shade(img);
			cout << "Done" << endl;
		}

		//Generate a set of brushfire local maxima, loMa,
		// from the brushfire map with doors being obstacles
		unordered_set<pos_t> loMa = getLocalMaxima<norm2BrushfireMap>
				(configurationSpace,norm2BrushfireDoors,reachable_coordinate,true);

		if(makePGMs) {
			cout << "Saving local maxima as \"floor_sweep_local_maxima.pgm\"... " << flush;
			for(auto c:loMa)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("floor_sweep_local_maxima.pgm");
			original.shade(img);
			cout << "Done." << endl;
		}

		//Insert some of the brushfire local maxima into the
		// set named coords (currently holding the brushfire "edges")
		chooseSubset(loMa,coords,robot_dynamics_radius);

		if(makePGMs) {
			cout << "Saving brushfire edges and local maxima as \"floor_sweep_be_and_loma.pgm\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("floor_sweep_be_and_loma.pgm");
			original.shade(img);
			cout << "Done." << endl;
		}

		//Iterating through the reachable coordinates (the configuration space),
		// find all the coordinates that would not be scanned by the robot
		// if it visited all points in coords,
		// and add these coordinates to coords on the way.
		// These remaining coordinates are called remainders.
		getRemainders(configurationSpace,coords,reachable_coordinate,robot_dynamics_radius);

		if(makePGMs) {
			ostringstream filename;
			filename << "floor_sweep_coordinate_set_"
					 << (coords.size()) << ".pgm";
			cout << "Saving floor sweeping coordinates as \"" << (filename.str()) << "\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM(filename.str());
			original.shade(img);
			cout << "Done." << endl;
		}

		return move(coords);
	}

	/**
	 * @brief sweep_floor Sweeps the floor
	 * @param img The original image
	 * @param makePGMs True if you want a lot of .pgm file output. False if you don't.
	 */
	static void sweep_floor(shared_ptr<Image> img, bool makePGMs=false)
	{
		//A reachable freespace coordinate is the starting coordinate
		const pos_t start = {ROBOT_START_X,ROBOT_START_Y};
		mapWrap maps(img,start,ROBOT_DYNAMICS_RADIUS);

		if(makePGMs) {
			cout << "Saving configuration space as \"floor_sweep_configuration_space.pgm\"... " << flush;
			(maps.getConfigurationSpace())->shade(img);
			img->saveAsPGM("floor_sweep_configuration_space.pgm");
			(maps.getOriginal())->shade(img);
			cout << "Done." << endl;
		}

		unordered_set<pos_t> coords =
				getFloorSweepCoordinates(img,
										 *(maps.getOriginal()),
										 maps.getFreespace(),
										 *(maps.getConfigurationSpace()),
										 *(maps.getNorm2BrushfireDoors()),
										 start,
										 ROBOT_DYNAMICS_RADIUS,
										 makePGMs);

		//The full path traveled by the robot is robotPath
		list<pos_t> robotPath;
		//The robot path starts with start.
		robotPath.push_back(start);

		auto n = coords.size();

		//The progress variable will be a measure (in percent) of
		// how many of the coordinates in coords (of size n)
		// the robot has visited.
		// The initial value must not be 0
		// (the reason is easy to see in the following code).
		size_t progress = 1;

		cout << "Starting robot movement. Points to visit: " << n << "." << endl;
		while(!(coords.empty())) {
			//First thing we do is output the progress to the console.
			auto lastProgress = progress;
			progress =(100*(n-coords.size()))/n;
			if(lastProgress!=progress) {
				cout << "\rPoints left in C: " << coords.size() << ", Traveled length: " << robotPath.size()
					 << ", Progress: " << progress << " %        " << flush;

				if(makePGMs) {
					ostringstream anim;
					anim << "floor_sweep_robot_path_"
						 << setw(3) << setfill('0')
						 << progress << ".pgm";
					img->saveAsPGM(anim.str());
				}
			}
			//The progress has now been output to the console.

			//Using a fast algorithm, we find the coordinate in coords,
			// that is closest to the robot's position ( robotPath.back() ) .
			// The optimal algorithm is Dijkstra's, but Wavefront is used
			// because it's faster/easier and almost as good.
			// The found coordinate is called "next".
			pos_t next = getClosestCoord(*(maps.getConfigurationSpace()),coords,robotPath.back());
			//getClosestCoord returns the input coordinate
			// if no coordinate was found in coords.
			// This should NOT happen if the preprocessing was done correctly.
			if(next==robotPath.back()) {
				cout << "\nError encountered. Saving remaining coordinates as \"floor_sweep_unreachable_coordinates.pgm\"..." << endl;
				(maps.getOriginal())->shade(img);
				for(auto c:coords)
					img->setPixel8U(c.cx(),c.cy(),87);
				img->saveAsPGM("floor_sweep_unreachable_coordinates.pgm");
				(maps.getOriginal())->shade(img);
				cout << "Done.\nTest has ended because of error." << endl;
				return;
			}

			//Remove the robot's current position from coords
			coords.erase(robotPath.back());
			//Remove the closest coordinate from coords
			coords.erase(next);

			//Using Dijkstra's algorithm, find all the coordinates between
			// the robot's current position and the closest coordinate in coords
			list<pos_t> pathToNext = (maps.getConfigurationSpace())->getDijkstraPath(robotPath.back(),next);

			//Add all the coordinates between the robot's current position
			// and the closest coordinate in coords
			// to the robot path and remove them from coords.
			for(auto c:pathToNext) {
				robotPath.push_back(c);
				if(makePGMs) {
					img->setPixel8U(c.cx(),c.cy(),20);
				}
				coords.erase(c);
			}
			//Now that all the coordinates between the robot's current position
			// and the closest coordinate in coords have been added to the robot path,
			// add the closest coordinate in coords to the path.
			robotPath.push_back(next);

			if(makePGMs) {
				img->setPixel8U(next.cx(),next.cy(),20);
			}
		}
		//The robot has now visited all coordinates in coords.
		// It's job is done.

		//Remove the progress bar
		cout << "\r                                                                " << endl;

		if(makePGMs) {
			cout << "Saving robot path as \"floor_sweep_robot_path_IDX.pgm\"... " << flush;
			ostringstream anim;
			anim << "floor_sweep_robot_path_full_" << setw(6) << setfill('0')
				 << robotPath.size() << "_" << n << ".pgm";
			for(auto v:robotPath)
				img->setPixel8U(v.cx(),v.cy(),20);
			img->saveAsPGM(anim.str());
			(maps.getOriginal())->shade(img);
			cout << "Done." << endl;
		}

		//Output the robot path length
		cout << "Total traveled length: " << robotPath.size() << endl;
	}


	static unordered_set<pos_t> getCupScanCoordinates(shared_ptr<Image> img,
														 const pixelshadeMap &original,
														 const unordered_set<pos_t> &freespace,
														 const pixelshadeMap &configurationSpace,
														 const norm2BrushfireMap &norm2BrushfireDoors,
														 const pos_t &reachable_coordinate,
														 size_t robot_scanner_radius=ROBOT_SCANNER_RADIUS)
	{
		return move(getFloorSweepCoordinates(img,original,freespace,configurationSpace,
										norm2BrushfireDoors,reachable_coordinate,robot_scanner_radius,
										false));
	}

	static void cup_scan(shared_ptr<Image> img, bool makePGMs=false)
	{
		simpleRobot robot(ROBOT_DYNAMICS_RADIUS, ROBOT_ARM_RADIUS,
						  ROBOT_SCANNER_RADIUS, ROBOT_CUP_CAPACITY,
						  ROBOT_SPEED_PIX_PER_H,
						  pos_t(ROBOT_START_X,ROBOT_START_Y));
		//A reachable freespace coordinate is the starting coordinate
		//const pos_t start = {ROBOT_START_X,ROBOT_START_Y};
		mapWrap maps(img,robot.start(),robot.dynamics_radius());
		pixelshadeMap cupspace(img);



//		for(coordIndexType x=0; x<coordIndexType(cupspace.getWidth()); ++x)
//			for(coordIndexType x=0; x<coordIndexType(cupspace.getWidth()); ++x)
//				cupspace.coordVal(x,y)=WSPACE_OBSTACLE;
//		for(auto v:(maps.getFreespace())) {
//			cupspace.coordVal(v)=(maps.originalMap->const_coordVal(v));
//		}

		unordered_set<pos_t> cupsToCollect;
		for(auto c:maps.getFreespace())
			if(WSPACE_IS_CUP(cupspace.const_coordVal(c)))
				cupsToCollect.insert(c);

		unordered_set<pos_t> cupsCollected;


		if(makePGMs) {
			cout << "Saving configuration space as \"cup_scan_configuration_space.pgm\"... " << flush;
			(maps.getConfigurationSpace())->shade(img);
			img->saveAsPGM("cup_scan_configuration_space.pgm");
			(maps.getOriginal())->shade(img);
			cout << "Done." << endl;
		}

		unordered_set<pos_t> coords =
				getCupScanCoordinates(img,
										 *(maps.getOriginal()),
										 maps.getFreespace(),
										 *(maps.getConfigurationSpace()),
										 *(maps.getNorm2BrushfireDoors()),
										 robot.start(),
										 robot.scanner_radius());

		if(makePGMs) {
			ostringstream filename;
			filename << "cup_scan_coordinate_set_"
					 << (coords.size()) << ".pgm";
			cout << "Saving cup scanner coordinate set as \"" << (filename.str()) << "\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM(filename.str());
			maps.getOriginal()->shade(img);
			cout << "Done." << endl;
		}		


		scanner cupscanner(robot.scanner_radius());
		scanner cuppicker(robot.arm_radius());
		size_t number_of_cups=0;

		set<pos_t> missed_cups;
		unordered_set<pos_t> reachableCSpace = maps.getConfigurationSpace()->findFreespace(robot.start());

		auto pickupCups = [&cupspace,&cupscanner,&cuppicker,&coords,
				&missed_cups,&number_of_cups,&reachableCSpace,
				&img,&makePGMs,&maps,&cupsCollected,&robot](const pos_t &c) {
			list<pos_t> visibleCups=
					cupscanner.scanlist(c,
										cupspace,
										robot.scanner_radius());
			list<pos_t> rc =
					cuppicker.scanlist(c,
									   cupspace,
									   robot.arm_radius());
			unordered_set<pos_t> reachableCups(rc.begin(),rc.end());
			for(auto cup:visibleCups) {
				if(number_of_cups<robot.cup_capacity()) {
					if(reachableCups.find(cup)!=reachableCups.end()) {
						++number_of_cups;
						cupspace.coordVal(cup)=WSPACE_FREE;
						cupsCollected.insert(cup);
						if(makePGMs) {
							img->setPixel8U(cup.cx(),cup.cy(),WSPACE_FREE);
						}
					}
					else
					{
						missed_cups.insert(cup);
						pos_t closest = cup;
						if(WSPACE_IS_OBSTACLE(maps.getConfigurationSpace()->const_coordVal(cup))) {
							closest=getClosestCoord(cupspace,reachableCSpace,cup);
							if(closest==cup)
								cout << "Unreachable cup at ( " << cup.cx() << " , " << cup.cy() << " )!" << endl;
						}
						coords.insert(closest);
					}
				}
				else {
					missed_cups.insert(cup);
					pos_t closest = cup;
					if(WSPACE_IS_OBSTACLE(maps.getConfigurationSpace()->const_coordVal(cup))) {
						closest=getClosestCoord(cupspace,reachableCSpace,cup);
						if(closest==cup)
							cout << "Unreachable cup at ( " << cup.cx() << " , " << cup.cy() << " )!" << endl;
					}
					coords.insert(closest);
				}
			}
		};

		cout << "Creating Dijkstra map from Offloading Stations... " << flush;
		dijkstraMap OLstations(img,dijkstraMap::getOffloadingStations(img));
		cout << "Done." << endl;


//		//The full path traveled by the robot is robotPath
//		list<pos_t> robotPath;
//		//The robot path starts with start.
//		robotPath.push_back(start);

		pickupCups(robot.currentPosition());
		auto n = coords.size();

		cout << "debug " << ((unsigned long)(std::ceil(robot.path_length()))) << endl;

		for(auto v:robot)
			cout << "debug ( " << v.cx() << " , " << v.cy() << " )" << endl;

		//The progress variable will be a measure (in percent) of
		// how many of the coordinates in coords (of size n)
		// the robot has visited.
		// The initial value must not be 0
		// (the reason is easy to see in the following code).
		size_t progress = 1;
		cout << "Starting robot movement. Points to visit: " << n << "." << endl;
		while(!(coords.empty())) {
			//First thing we do is output the progress to the console.
			auto lastProgress = progress;
			progress =(100*(n-coords.size()))/n;
			if(lastProgress!=progress) {
				cout << "\rPoints left in C: " << coords.size() << ", Traveled length: " << (robot.path_length()) //robotPath.size()
					 << ", Progress: " << progress << " %        " << flush;

				if(makePGMs) {
					ostringstream anim;
					anim << "cup_scan_robot_path_"
						 << setw(3) << setfill('0')
						 << progress << ".pgm";
					img->saveAsPGM(anim.str());
				}
			}
			//The progress has now been output to the console.

			//Using a fast algorithm, we find the coordinate in coords,
			// that is closest to the robot's position ( robotPath.back() ) .
			// The optimal algorithm is Dijkstra's, but Wavefront is used
			// because it's faster/easier and almost as good.
			// The found coordinate is called "next".
			pos_t next = getClosestCoord(*(maps.getConfigurationSpace()),coords,robot.currentPosition());
			//getClosestCoord returns the input coordinate
			// if no path was found to any coordinate in coords.
			// This should NOT happen if the preprocessing was done correctly.
			if(next==robot.currentPosition()) {
				cout << "\nError encountered. Saving remaining coordinates as \"cup_scan_unreachable_coordinates.pgm\"..." << endl;
				(maps.getOriginal())->shade(img);
				for(auto c:coords)
					img->setPixel8U(c.cx(),c.cy(),87);
				img->saveAsPGM("cup_scan_unreachable_coordinates.pgm");
				(maps.getOriginal())->shade(img);
				cout << "Done." << endl;
				cout << "There were " << (coords.size()) << " remaining coordinates." << endl;
				if(coords.size()>10)
					cout << "One of them was at ( " << ((*(coords.begin())).cx())
						<< " , " << ((*(coords.begin())).cy()) << " )." << endl;
				else {
					cout << "They are: " << endl;
					for(auto c:coords)
						cout << "( " << c.cx() << " , " << c.cy() << " ), ";
				}
				cout << "Test has ended because of error." << endl;
				return;
			}

			//Remove the robot's current position from coords
			coords.erase(robot.currentPosition());
			//Remove the closest coordinate from coords
			coords.erase(next);

			//Using Dijkstra's algorithm, find all the coordinates between
			// the robot's current position and the closest coordinate in coords
			list<pos_t> pathToNext = (maps.getConfigurationSpace())->getDijkstraPath(robot.currentPosition(),next);

			//Add all the coordinates between the robot's current position
			// and the closest coordinate in coords
			// to the robot path and remove them from coords.

			for(auto c:pathToNext) {
				robot.move(c);//robotPath.push_back(c);
				if(makePGMs) {
					img->setPixel8U(c.cx(),c.cy(),20);
				}
				coords.erase(c);

				pickupCups(c);
			}
			//Now that all the coordinates between the robot's current position
			// and the closest coordinate in coords have been added to the robot path,
			// add the closest coordinate in coords to the path.
			robot.move(next);//robotPath.push_back(next);
			if(makePGMs) {
				img->setPixel8U(next.cx(),next.cy(),20);
			}
			pickupCups(next);
			if(number_of_cups==robot.cup_capacity()) {
				list<pos_t> homepath = OLstations.getShortestPath(robot.currentPosition());
				for(auto c:homepath) {
					robot.move(c);//robotPath.push_back(c); //move one step towards home
					if(makePGMs) {
						img->setPixel8U(c.cx(),c.cy(),20);
					}
					coords.erase(c);
					pickupCups(c);
				}
				for(auto cup:missed_cups) {
					pos_t closest = cup;
					if(WSPACE_IS_OBSTACLE(maps.getConfigurationSpace()->const_coordVal(cup))) {
						closest=getClosestCoord(cupspace,reachableCSpace,cup);
						if(closest==cup)
							cout << "Unreachable cup at ( " << cup.cx() << " , " << cup.cy() << " )!" << endl;
					}
					coords.insert(closest);
				}
				missed_cups.clear();
				number_of_cups=0;
			}
		}
		//The robot has now visited all coordinates in coords.
		// It's job is done.

		//Remove the progress bar
		cout << "\r                                                                " << endl;

		if(makePGMs) {
			cout << "Saving robot path as \"cup_scan_robot_path_IDX.pgm\".." << flush;
			ostringstream anim;
			anim << "cup_scan_robot_path_full_" << setw(8) << setfill('0')
				 << ((unsigned long)(std::ceil(robot.path_length()))) << "_" << n << ".pgm";
			cout << ". " << flush;
			for(auto v:robot)
				img->setPixel8U(v.cx(),v.cy(),20);
			img->saveAsPGM(anim.str());
			(maps.getOriginal())->shade(img);
			cout << "Done." << endl;
		}


		//Output the cup results
		cout << "Collected " << (cupsCollected.size())
			 << " out of " << (cupsToCollect.size())
			 << " cups." << endl;
		if(cupsToCollect.size() != cupsCollected.size()) {
			cout << "The remaining " << (cupsToCollect.size() - cupsCollected.size())
				 << " cups have the following coordinates:" << endl;
			for(auto cup:cupsToCollect)
				if(cupsCollected.find(cup)!=cupsCollected.end())
					cout << "( " << cup.cx() << " , " << cup.cy() << " )  ";
			cout << endl;
		}
		//Output the robot path length
		cout << "Total traveled length: " << robot.path_length() << endl;
		cout << "Total travel time: " << robot.path_time_hours() << " hours" << endl;

	}

};
