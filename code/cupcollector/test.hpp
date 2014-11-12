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


using namespace std;

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
	 * @brief chooseLocalMaxima Adds some of the coordinates from the local maxima to the coordSet.
	 * @param localMaxima The local maxima
	 * @param coordSet The coordinate set we want to expand to include some more (unscanned) coordinates.
	 * @param radius The radius to scan around each local maximum.
	 */
	static void chooseLocalMaxima(const unordered_set<pos_t> &localMaxima,
								  unordered_set<pos_t> &coordSet,
								  size_t radius=ROBOT_DYNAMICS_RADIUS) {
		for(auto c:localMaxima)
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
	 * @brief test Performs the experiment
	 * @param img The original image
	 * @param banim True if you want a lot of .pgm file output. False if you don't.
	 */
	static void test(shared_ptr<Image> img, bool banim=true)
	{
		//A reachable freespace coordinate is the starting coordinate
		const pos_t start = {ROBOT_START_X,ROBOT_START_Y};
		//Creating internal representaion (and backup) named theImg,
		// and a map to represent the configuration space named configurationSpace
		pixelshadeMap theImg(img), configurationSpace(img);
		//Standard integer brushfire map
		brushfireMap brush(theImg,start);

		//Create the configuration space by padding
		{
			norm2BrushfireMap n2Brush(theImg);
			pad<norm2BrushfireMap>(configurationSpace,n2Brush,ROBOT_DYNAMICS_RADIUS);
		}

		if(banim) {
			cout << "Saving configuration space as \"test_configuration_space.pgm\"... " << flush;
			configurationSpace.shade(img);
			img->saveAsPGM("test_configuration_space.pgm");
			theImg.shade(img);
			cout << "Done." << endl;
		}

		//Detect the door steps (in the integer brushfire map)
		doorDetector mydetective;
		cout << "Finding The Doors... " << flush;
		vector<pos_t> The_Doors = mydetective.detect_doorways(img, brush);
		cout << "Done.\nFinding Door Steps... " << flush;
		pixelshade_map door_steps_map = mydetective.door_step(img, brush, The_Doors);
		The_Doors.clear();

		//Create a norm2 brushfire map, counting door steps as obstacles
		cout << "Done.\nCreating brushfire map from door steps... " << flush;
		norm2BrushfireMap doorBrush(door_steps_map);
		cout << "Done." << endl;

		//Find the freespace
		unordered_set<pos_t> free = theImg.findFreespace(start);
		//Go through the brushfire map and generate a set, coords, of all the coordinates
		// with brushfire values equal to certain multiples of robot
		// diameter/radius combinations.
		// This set will be called the brushfire "edges".
		unordered_set<pos_t> coords = getBrushEdges(free,doorBrush,ROBOT_DYNAMICS_RADIUS);

		if(banim) {
			cout << "Saving brushfire edges as \"test_brushfire_edges.pgm\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("test_brushfire_edges.pgm");
			theImg.shade(img);
			cout << "Done" << endl;
		}

		//Generate a set of brushfire local maxima, loMa,
		// from the brushfire map with doors being obstacles
		unordered_set<pos_t> loMa = getLocalMaxima<norm2BrushfireMap>(configurationSpace,doorBrush,start,true);

		if(banim) {
			cout << "Saving local maxima as \"test_local_maxima.pgm\"... " << flush;
			for(auto c:loMa)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("test_local_maxima.pgm");
			theImg.shade(img);
			cout << "Done." << endl;
		}

		//Insert some of the brushfire local maxima into the
		// set named coords (currently holding the brushfire "edges")
		chooseLocalMaxima(loMa,coords,ROBOT_DYNAMICS_RADIUS);

		if(banim) {
			cout << "Saving brushfire edges and local maxima as \"test_be_and_loma.pgm\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("test_be_and_loma.pgm");
			theImg.shade(img);
			cout << "Done." << endl;
		}

		//Iterating through the reachable coordinates (the configuration space),
		// find all the coordinates that would not be scanned by the robot
		// if it visited all points in coords,
		// and add these coordinates to coords on the way.
		// These remaining coordinates are called remainders.
		getRemainders(configurationSpace,coords,start,ROBOT_DYNAMICS_RADIUS);

		if(banim) {
			cout << "Saving brushfire edges, local maxima and remainders as \"test_full_coordinate_set.pgm\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("test_full_coordinate_set.pgm");
			theImg.shade(img);
			cout << "Done." << endl;
		}

		//The full path traveled by the robot is robotPath
		list<pos_t> robotPath;
		//The robot path starts with start.
		robotPath.push_back(start);

		//n is the number of coordinates the robot MUST visit.
		auto n = coords.size();
		//The progress variable will be a measure (in percent) of
		// how many of the coordinates in coords (of size n)
		// the robot has visited.
		// The initial value must not be 0
		// (the reason is easy to see in the following code).
		size_t progress = 1;

		cout << "Starting robot movement. Points to visit (C): " << n << "." << endl;
		while(!(coords.empty())) {
			//First thing we do is output the progress to the console.
			auto lastProgress = progress;
			progress =(100*(n-coords.size()))/n;
			if(lastProgress!=progress) {
				cout << "\rPoints left in C: " << coords.size() << ", Traveled length: " << robotPath.size()
					 << ", Progress: " << progress << " %        " << flush;

				if(banim) {
					ostringstream anim;
					anim << "test_robot_path_"
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
			pos_t next = getClosestCoord(configurationSpace,coords,robotPath.back());
			//getClosestCoord returns the input coordinate
			// if no coordinate was found in coords.
			// This should NOT happen if the preprocessing was done correctly.
			if(next==robotPath.back()) {
				cout << "\nError encountered. Saving remaining coordinates as \"test_unreachable_coordinates.pgm\"..." << endl;
				theImg.shade(img);
				for(auto c:coords)
					img->setPixel8U(c.cx(),c.cy(),87);
				img->saveAsPGM("test_unreachable_coordinates.pgm");
				theImg.shade(img);
				cout << "Done.\nTest has ended because of error." << endl;
				return;
			}

			//Remove the robot's current position from coords
			coords.erase(robotPath.back());
			//Remove the closest coordinate from coords
			coords.erase(next);

			//Using Dijkstra's algorithm, find all the coordinates between
			// the robot's current position and the closest coordinate in coords
			list<pos_t> pathToNext = configurationSpace.getDijkstraPath(robotPath.back(),next);

			//Add all the coordinates between the robot's current position
			// and the closest coordinate in coords
			// to the robot path and remove them from coords.
			for(auto c:pathToNext) {
				robotPath.push_back(c);
				if(banim) {
					img->setPixel8U(c.cx(),c.cy(),20);
				}
				coords.erase(c);
			}
			//Now that all the coordinates between the robot's current position
			// and the closest coordinate in coords have been added to the robot path,
			// add the closest coordinate in coords to the path.
			robotPath.push_back(next);

			if(banim) {
				img->setPixel8U(next.cx(),next.cy(),20);
			}
		}
		//The robot has now visited all coordinates in coords.
		// It's job is done.

		//Remove the progress bar
		cout << "\r                                                                " << endl;

		if(banim) {
			cout << "Saving robot path as \"test_robot_path_IDX.pgm\"... " << flush;
			ostringstream anim;
			anim << "test_robot_path_full_" << setw(6) << setfill('0')
				 << robotPath.size() << "_" << n << ".pgm";
			for(auto v:robotPath)
				img->setPixel8U(v.cx(),v.cy(),20);
			img->saveAsPGM(anim.str());
			theImg.shade(img);
			cout << "Done." << endl;
		}

		//Output the robot path length
		cout << "Total traveled length: " << robotPath.size() << endl;
	}
};
