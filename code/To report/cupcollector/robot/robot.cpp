/**
 * @file robot.cpp
 * @author Mikael Westermann
 */
#include "robot.h"
#include <forward_list>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <chrono>




using namespace std;


template<typename BrushmapT>
unordered_set<pos_t> robot::getBrushEdges(const unordered_set<pos_t> &freespace, const BrushmapT &brush, size_t radius)
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

pos_t robot::getClosestCoord(const pixelshadeMap &img,
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
void robot::pad(pixelshadeMap &imgToPad, const BrushmapT &brush, const size_t padValue) {
	for(coordIndexType x=0; x<(coordIndexType)imgToPad.getWidth(); ++x)
		for(coordIndexType y=0; y<(coordIndexType)imgToPad.getHeight(); ++y)
			if(brush.const_coordVal(x,y) <= (typename BrushmapT::myValType)(padValue))
				imgToPad.coordVal(x,y) = WSPACE_OBSTACLE;
}

template<typename BrushmapT>
unordered_set<pos_t> robot::getLocalMaxima(const pixelshadeMap &img,
												  const BrushmapT &brush,
												  const pos_t &validFreespaceCoord,
												  bool twoPointMaxima)
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

void robot::chooseSubset(const unordered_set<pos_t> &chooseFrom,
								unordered_set<pos_t> &coordSet,
								size_t radius) {
	for(auto c:chooseFrom)
		if(isRemainder(c,coordSet,radius))
			coordSet.insert(c);
}

bool robot::isRemainder(pos_t center, const unordered_set<pos_t> &coords, unsigned int radius)
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

void robot::getRemainders(const pixelshadeMap &img,
								 unordered_set<pos_t> &coordSet,
								 const pos_t &freespaceCoord,
								 size_t radius) {
	unordered_set<pos_t> freespace = img.findFreespace(freespaceCoord);
	for(auto c:freespace)
		if(isRemainder(c,coordSet,radius))
			coordSet.insert(c);
}


unordered_set<pos_t> robot::getFloorSweepCoordinates(shared_ptr<Image> img,
															const pixelshadeMap &original,
															const unordered_set<pos_t> &freespace,
															const pixelshadeMap &configurationSpace,
															const norm2BrushfireMap &norm2BrushfireDoors,
															const pos_t &reachable_coordinate,
															size_t robot_dynamics_radius,
															bool makePGMs)
{
	//Go through the brushfire map and generate a set, coords, of all the coordinates
	// with brushfire values equal to certain multiples of robot
	// diameter/radius combinations.
	// This set will be called the brushfire "edges".
	unordered_set<pos_t> coords = getBrushEdges(freespace,norm2BrushfireDoors,robot_dynamics_radius);
	if(makePGMs) {
		cout << " Saving brushfire edges as \"floor_sweep_brushfire_edges.pgm\"... " << flush;
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
		cout << " Saving local maxima as \"floor_sweep_local_maxima.pgm\"... " << flush;
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
		cout << " Saving brushfire edges and local maxima as \"floor_sweep_be_and_loma.pgm\"... " << flush;
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
		cout << " Saving floor sweeping coordinates as \"" << (filename.str()) << "\"... " << flush;
		for(auto c:coords)
			img->setPixel8U(c.cx(),c.cy(),20);
		img->saveAsPGM(filename.str());
		original.shade(img);
		cout << "Done." << endl;
	}

	return move(coords);
}



unordered_set<pos_t> robot::getCupScanCoordinates(shared_ptr<Image> img,
														 const pixelshadeMap &original,
														 const unordered_set<pos_t> &freespace,
														 const pixelshadeMap &configurationSpace,
														 const norm2BrushfireMap &norm2BrushfireDoors,
														 const pos_t &reachable_coordinate,
														 size_t robot_scanner_radius)
{
	return move(getFloorSweepCoordinates(img,original,freespace,configurationSpace,
										 norm2BrushfireDoors,reachable_coordinate,robot_scanner_radius,
										 false));
}


void robot::pickupCups(pixelshadeMap &cupspace, unordered_set<pos_t> &coordSet,
					   set<pos_t> &missed_cups, const unordered_set<pos_t> &reachableCSpace,
					   shared_ptr<Image> img, bool makePGMs,
					   unordered_set<pos_t> &cupsCollected, const pos_t &c)
{
	list<pos_t> visibleCups= cupscanner.scanlistLineOfSight(c,cupspace);
	list<pos_t> rc = cuppicker.scanlistAroundWalls(c,cupspace);
	unordered_set<pos_t> reachableCups(rc.begin(),rc.end());
	for(auto cup:visibleCups) {
		if(number_of_cups<cupCap) {
			if(reachableCups.find(cup)!=reachableCups.end()) {
				++number_of_cups;
				cupspace.coordVal(cup)=WSPACE_FREE;
				cupsCollected.insert(cup);
				if(makePGMs)
					img->setPixel8U(cup.cx(),cup.cy(),WSPACE_FREE);
			}
			else {
				missed_cups.insert(cup);
				pos_t closest = cup;
				if(WSPACE_IS_OBSTACLE(myMaps.getConfigurationSpace()->const_coordVal(cup))) {
					closest=getClosestCoord(cupspace,reachableCSpace,cup);
					if(closest==cup)
						cout << " Unreachable cup at " << cup << "!" << endl;
				}
				coordSet.insert(closest);
			}
		}
		else {
			missed_cups.insert(cup);
			pos_t closest = cup;
			if(WSPACE_IS_OBSTACLE(myMaps.getConfigurationSpace()->const_coordVal(cup))) {
				closest=getClosestCoord(cupspace,reachableCSpace,cup);
				if(closest==cup)
					cout << " Unreachable cup at " << cup << "!" << endl;
			}
			coordSet.insert(closest);
		}
	}
}




robot::robot(shared_ptr<Image> img,
			 size_t radius_dynamics, size_t radius_arm, size_t radius_scanner,
			 size_t cup_capacity, double speed_pix_pr_h, pos_t starting_position)
	:r_dynamics(radius_dynamics), cupCap(cup_capacity),
	  cupscanner(scanner(radius_scanner)),cuppicker(scanner(radius_arm)),
	  number_of_cups(0),
	  myMoves(movement(speed_pix_pr_h,starting_position))
{
	auto startT = std::chrono::system_clock::now();
	myMaps=mapWrap(img,starting_position,radius_dynamics);
	auto duraT = (chrono::duration_cast<chrono::milliseconds>
				  (chrono::system_clock::now()-startT)).count();
	cout << " mapWrap generation took " << duraT << " ms." << endl;
}



void robot::sweep_floor(shared_ptr<Image> img, bool makePGMs)
{
	myMoves.clearPath(myMoves.start());
	auto startT = std::chrono::system_clock::now();
	auto duraT = (chrono::duration_cast<chrono::milliseconds>
				  (chrono::system_clock::now()-startT)).count();


	if(makePGMs) {
		cout << " Saving configuration space as \"floor_sweep_configuration_space.pgm\"... " << flush;
		(myMaps.getConfigurationSpace())->shade(img);
		img->saveAsPGM("floor_sweep_configuration_space.pgm");
		(myMaps.getOriginal())->shade(img);
		cout << "Done." << endl;
	}






	startT = std::chrono::system_clock::now();

	unordered_set<pos_t> coords =
			getFloorSweepCoordinates(img,
									 *(myMaps.getOriginal()),
									 myMaps.getFreespace(),
									 *(myMaps.getConfigurationSpace()),
									 *(myMaps.getNorm2BrushfireDoors()),
									 myMoves.start(),
									 r_dynamics,
									 makePGMs);

	duraT = (chrono::duration_cast<chrono::milliseconds>
			 (chrono::system_clock::now()-startT)).count();
	cout << " getFloorSweepCoordinates took " << duraT << " ms." << endl;





	auto n = coords.size();

	//The progress variable will be a measure (in percent) of
	// how many of the coordinates in coords (of size n)
	// the robot has visited.
	// The initial value must not be 0
	// (the reason is easy to see in the following code).
	size_t progress = 1;



	startT = std::chrono::system_clock::now();


	cout << " Starting robot movement. Points to visit: " << n << "." << endl;
	while(!(coords.empty())) {
		//First thing we do is output the progress to the console.
		auto lastProgress = progress;
		progress =(100*(n-coords.size()))/n;
		if(lastProgress!=progress) {
			cout << "\r Points left in C: " << coords.size() << ", Traveled length: " << (myMoves.path_length()/10.0) << " m"
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
		// that is closest to the robot's position.
		// The optimal algorithm is Dijkstra's, but Wavefront is used
		// because it's faster/easier and almost as good.
		// The found coordinate is called "next".
		pos_t next = getClosestCoord(*(myMaps.getConfigurationSpace()),coords,myMoves.currentPosition());
		//getClosestCoord returns the input coordinate
		// if no coordinate was found in coords.
		// This should NOT happen if the preprocessing was done correctly.
		if(next==myMoves.currentPosition()) {
			cout << "\n Error encountered. Saving remaining coordinates as \"floor_sweep_unreachable_coordinates.pgm\"..." << endl;
			(myMaps.getOriginal())->shade(img);
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),87);
			img->saveAsPGM("floor_sweep_unreachable_coordinates.pgm");
			(myMaps.getOriginal())->shade(img);
			cout << "Done.\nTest has ended because of error." << endl;
			return;
		}

		//Remove the robot's current position from coords
		coords.erase(myMoves.currentPosition());
		//Remove the closest coordinate from coords
		coords.erase(next);

		//Using Dijkstra's algorithm, find all the coordinates between
		// the robot's current position and the closest coordinate in coords
		list<pos_t> pathToNext = (myMaps.getConfigurationSpace())->getDijkstraPath(myMoves.currentPosition(),next);

		//Add all the coordinates between the robot's current position
		// and the closest coordinate in coords
		// to the robot path and remove them from coords.
		for(auto c:pathToNext) {
			myMoves.move(c);
			if(makePGMs)
				img->setPixel8U(c.cx(),c.cy(),20);
			coords.erase(c);
		}
		//Now that all the coordinates between the robot's current position
		// and the closest coordinate in coords have been added to the robot path,
		// add the closest coordinate in coords to the path.
		myMoves.move(next);

		if(makePGMs)
			img->setPixel8U(next.cx(),next.cy(),20);


		//Make sure it returns to starting position
		if(coords.empty() && ( myMoves.currentPosition() != myMoves.start() ) )
			coords.insert(myMoves.start());


	}
	//The robot has now visited all coordinates in coords.
	// It's job is done.

	//Remove the progress bar
	cout << "\r                                                                " << flush << "\r";



	duraT = (chrono::duration_cast<chrono::milliseconds>
			 (chrono::system_clock::now()-startT)).count();
	cout << " Robot traveling took " << duraT << " ms." << endl;






	if(makePGMs) {
		cout << " Saving robot path as \"floor_sweep_robot_path_IDX.pgm\"... " << flush;
		ostringstream anim;
		anim << "floor_sweep_robot_path_full_" << setw(6) << setfill('0')
			 << ((size_t)std::ceil(myMoves.path_length())) << "_" << n << ".pgm";
		for(auto v:myMoves)
			img->setPixel8U(v.cx(),v.cy(),20);
		img->saveAsPGM(anim.str());
		(myMaps.getOriginal())->shade(img);
		cout << "Done." << endl;
	}

	//Output the robot path length
	cout << " Total traveled length: " << (myMoves.path_length()/10.0) << " meters." << endl;
}


void robot::cup_scan(shared_ptr<Image> img, bool makePGMs)
{
	myMoves.clearPath(myMoves.start());

	auto startT = std::chrono::system_clock::now();
	auto duraT = (chrono::duration_cast<chrono::milliseconds>
				  (chrono::system_clock::now()-startT)).count();

	pixelshadeMap cupspace(img);

	unordered_set<pos_t> cupsToCollect;
	for(auto c:myMaps.getFreespace())
		if(WSPACE_IS_CUP(cupspace.const_coordVal(c)))
			cupsToCollect.insert(c);
	unordered_set<pos_t> cupsCollected;


	if(makePGMs) {
		cout << " Saving configuration space as \"cup_scan_configuration_space.pgm\"... " << flush;
		(myMaps.getConfigurationSpace())->shade(img);
		img->saveAsPGM("cup_scan_configuration_space.pgm");
		(myMaps.getOriginal())->shade(img);
		cout << "Done." << endl;
	}






	startT = std::chrono::system_clock::now();

	unordered_set<pos_t> coords =
			getCupScanCoordinates(img,
								  *(myMaps.getOriginal()),
								  myMaps.getFreespace(),
								  *(myMaps.getConfigurationSpace()),
								  *(myMaps.getNorm2BrushfireDoors()),
								  myMoves.start(),
								  cupscanner.radius());

	duraT = (chrono::duration_cast<chrono::milliseconds>
			 (chrono::system_clock::now()-startT)).count();
	cout << " getCupScanCoordinates took " << duraT << " ms." << endl;






	if(makePGMs) {
		ostringstream filename;
		filename << "cup_scan_coordinate_set_"
				 << (coords.size()) << ".pgm";
		cout << " Saving cup scanner coordinate set as \"" << (filename.str()) << "\"... " << flush;
		for(auto c:coords)
			img->setPixel8U(c.cx(),c.cy(),20);
		img->saveAsPGM(filename.str());
		myMaps.getOriginal()->shade(img);
		cout << "Done." << endl;
	}



	number_of_cups=0;

	set<pos_t> missed_cups;
	unordered_set<pos_t> reachableCSpace = myMaps.getConfigurationSpace()->findFreespace(myMoves.start());





	auto pickupCupsLocal = [&cupspace,&coords,
			&missed_cups,&reachableCSpace,
			&img,&makePGMs,&cupsCollected,this](const pos_t &c) {
		pickupCups(cupspace,coords,missed_cups,reachableCSpace,img,makePGMs,cupsCollected,c);
	};

	cout << " Creating Dijkstra map from Offloading Stations... " << flush;
	dijkstraMap OLstations(img,dijkstraMap::getOffloadingStations(img));
	cout << "Done." << endl;


	pickupCupsLocal(myMoves.currentPosition());
	auto n = coords.size();





	startT = std::chrono::system_clock::now();



	//The progress variable will be a measure (in percent) of
	// how many of the coordinates in coords (of size n)
	// the robot has visited.
	// The initial value must not be 0
	// (the reason is easy to see in the following code).
	size_t progress = 1;
	cout << " Starting robot movement. Points to visit: " << n << "." << endl;
	while(!(coords.empty())) {
		//First thing we do is output the progress to the console.
		auto lastProgress = progress;
		progress =(100*(n-coords.size()))/n;
		if(lastProgress!=progress) {
			cout << "\r Points left in C: " << coords.size() << ", Traveled length: " << (myMoves.path_length()/10.0) << " m"
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
		// that is closest to the robot's position.
		// The optimal algorithm is Dijkstra's, but Wavefront is used
		// because it's faster/easier and almost as good.
		// The found coordinate is called "next".
		pos_t next = getClosestCoord(*(myMaps.getConfigurationSpace()),coords,myMoves.currentPosition());
		//getClosestCoord returns the input coordinate
		// if no path was found to any coordinate in coords.
		// This should NOT happen if the preprocessing was done correctly.
		if(next==myMoves.currentPosition()) {
			cout << "\n Error encountered. Saving remaining coordinates as \"cup_scan_unreachable_coordinates.pgm\"..." << endl;
			(myMaps.getOriginal())->shade(img);
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),87);
			img->saveAsPGM("cup_scan_unreachable_coordinates.pgm");
			(myMaps.getOriginal())->shade(img);
			cout << "Done." << endl;
			cout << " There were " << (coords.size()) << " remaining coordinates." << endl;
			if(coords.size()>10)
				cout << " One of them was at "<< (*(coords.begin())) << "." << endl;
			else {
				cout << " They are: " << endl;
				for(auto c:coords)
					cout << c << ", ";
			}
			cout << "Test has ended because of error." << endl;
			return;
		}

		//Remove the robot's current position from coords
		coords.erase(myMoves.currentPosition());
		//Remove the closest coordinate from coords
		coords.erase(next);

		//Using Dijkstra's algorithm, find all the coordinates between
		// the robot's current position and the closest coordinate in coords
		list<pos_t> pathToNext = (myMaps.getConfigurationSpace())->getDijkstraPath(myMoves.currentPosition(),next);

		//Add all the coordinates between the robot's current position
		// and the closest coordinate in coords
		// to the robot path and remove them from coords.

		for(auto c:pathToNext) {
			myMoves.move(c);
			if(makePGMs) {
				img->setPixel8U(c.cx(),c.cy(),20);
			}
			coords.erase(c);

			pickupCupsLocal(c);
		}
		//Now that all the coordinates between the robot's current position
		// and the closest coordinate in coords have been added to the robot path,
		// add the closest coordinate in coords to the path.
		myMoves.move(next);
		if(makePGMs) {
			img->setPixel8U(next.cx(),next.cy(),20);
		}
		pickupCupsLocal(next);
		if(number_of_cups==cupCap) {
			list<pos_t> homepath = OLstations.getShortestPath(myMoves.currentPosition());
			for(auto c:homepath) {
				myMoves.move(c); //move one step towards home
				if(makePGMs) {
					img->setPixel8U(c.cx(),c.cy(),20);
				}
				coords.erase(c);
				pickupCupsLocal(c);
			}
			for(auto cup:missed_cups) {
				pos_t closest = cup;
				if(WSPACE_IS_OBSTACLE(myMaps.getConfigurationSpace()->const_coordVal(cup))) {
					closest=getClosestCoord(cupspace,reachableCSpace,cup);
					if(closest==cup)
						cout << " Unreachable cup at " << cup << "!" << endl;
				}
				coords.insert(closest);
			}
			missed_cups.clear();
			number_of_cups=0;
		}


		//Make sure robot returns to starting position when finished
		if( coords.empty() && (myMoves.currentPosition() != myMoves.start()) )
			coords.insert(myMoves.start());


	}
	//The robot has now visited all coordinates in coords.
	// It's job is almost done.






	//Remove the progress bar
	cout << "\r                                                                " << flush << "\r";


	duraT = (chrono::duration_cast<chrono::milliseconds>
			 (chrono::system_clock::now()-startT)).count();
	cout << " Robot traveling took " << duraT << " ms." << endl;





	if(makePGMs) {
		cout << " Saving robot path as \"cup_scan_robot_path_IDX.pgm\"... " << flush;
		ostringstream anim;
		anim << "cup_scan_robot_path_full_" << setw(6) << setfill('0')
			 << ((unsigned long)(std::ceil(myMoves.path_length()))) << "_" << n << ".pgm";
		for(auto v:myMoves)
			img->setPixel8U(v.cx(),v.cy(),20);
		img->saveAsPGM(anim.str());
		(myMaps.getOriginal())->shade(img);
		cout << "Done." << endl;
	}


	//Output the cup results
	cout << " Collected " << (cupsCollected.size())
		 << " out of " << (cupsToCollect.size())
		 << " cups." << endl;
	if(cupsToCollect.size() != cupsCollected.size()) {
		cout << " The remaining " << (cupsToCollect.size() - cupsCollected.size())
			 << " cups have the following coordinates:" << endl << " ";
		for(auto cup:cupsToCollect)
			if(cupsCollected.find(cup)!=cupsCollected.end())
				cout << cup << "  ";
		cout << endl;
	}
	//Output the robot path length
	cout << " Total traveled length: " << (myMoves.path_length()/10.0) << " meters" << endl;
	cout << " Total travel time: "
		 << ((unsigned int)(std::floor(myMoves.path_time_hours()))) << " hours "
		 << ((unsigned int)(std::ceil( ( myMoves.path_time_hours() - (std::floor(myMoves.path_time_hours())) )*60 )))
		 << " minutes." << endl;

}
