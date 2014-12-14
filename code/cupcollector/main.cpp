/** @file
* @author Lukas, Michael, Mikkel, Nikolaj & Mikael
* @date 21-10-2014
*
* Group project.
*/
//Use <stdheader> to include standard headers.
//Use "relative/path/to/myOwnHeader.h" to include your own headers.
//Then it'll work on both GCC and MS Visual C++ compilers.
#include <iostream>
#include "libraries/Image.hpp"
#include "libraries/PPMLoader.hpp"
#include <memory>
#include "tekmap/tekmap.hpp"
#include "tekmap/pixelshade.h"
#include "doordetector/doordetector.h"
#include "scanner/scanner.h"
#include "robot/robot.h"
using namespace rw::sensor;
using namespace rw::loaders;
using namespace std;
void testTekMapConstructors(const string &filename)
{
	pos_t start = {ROBOT_START_X,ROBOT_START_Y};
	cout << "Loading image..." << endl;
	shared_ptr<Image> img(PPMLoader::load(filename));
	shared_ptr<Image> canvas(PPMLoader::load(filename));
	cout << "Image size: " << img->getWidth()
		 << " x " << img->getHeight() << endl;
	//Store original image:
	pixelshade_map original(img);
	//Run brushfire algorithm on reachable freespace:
	brushfire_map brush(original,start);
	//brushfire_map brush(img,brushfire_map::BRUSHFIRE);
	//Paint the brushfire values onto the canvas:
	brush.shade(canvas);
	//Save the brushfire painting:
	canvas->saveAsPGM("brushfire.pgm");
	//Run wavefront algorithm on reachable freespace, goals being the Offloading stations:
	wavefront_map wave(img,wavefront_map::getOffloadingStations(img));
	//Paint the wavefront values onto the canvas:
	wave.shade(canvas);
	//Save the wavefront painting:
	canvas->saveAsPGM("wavefront.pgm");
	//Turn the painted canvas back to the original:
	original.shade(canvas);
	//Save the original painting:
	canvas->saveAsPGM("original.pgm");
	dijkstraMap dij(img,dijkstraMap::getOffloadingStations(img));
	dij.shade(canvas);
	canvas->saveAsPGM("dijkstra.pgm");
}

void testShortestPath(shared_ptr<Image> img)
{
	pos_t start = {ROBOT_START_X,ROBOT_START_Y};
	pos_t goal = {2710,1292};
	cout << "Constructing tekmaps" << endl;
	wavefront_map wave(img,wavefront_map::getOffloadingStations(img));
	pixelshadeMap pix(img);

	cout << "Getting shortest path..." << endl;
	list<pos_t> path1 = wave.getWavefrontPath(img,start,goal);
	//for(auto i:path1) cout << "( " << i.cx() << " , " << i.cy() << " )" << endl;
	cout << "Done" << endl;
	cout << "One more time!" << endl;
	list<pos_t> path2 = pix.getWavefrontPath(start,goal);
	//for(auto i:path2) cout << "( " << i.cx() << " , " << i.cy() << " )" << endl;
	cout << (path1==path2?":-)":":-(") << endl;

	list<pos_t> path3 = pix.getDijkstraPath(start,goal);
	for(auto i:path3) cout << "( " << i.cx() << " , " << i.cy() << " )" << endl;
	cout << "Dijkstra length: " << path3.size()
		 << " vs. wavefront length: " << path2.size() << endl;

	list<pos_t> path4 = wave.getDijkstraPath(img,start,goal);
	for(auto i:path4) cout << "( " << i.cx() << " , " << i.cy() << " )" << endl;
	cout << "Dijkstra length: " << path4.size() << (path3==path4?":-)":":-(") << endl;
}


void test_doors(shared_ptr<Image> img, const pos_t &robot_start) {
	pixelshade_map original(img );
	cout << "Creating brushfire with start position..." << endl;
	brushfire_map brush(img , robot_start);
	cout << "brushfire done..." << endl;
	doorDetector mydetective;
	cout << "Finding The Doors " << endl;
	vector<pos_t> The_Doors = mydetective.detect_doorways(img, brush);
	cout << "Finding Door Steps" << endl;
	pixelshade_map door_steps_map = mydetective.door_step(img, brush, The_Doors);
	brush.shade(img);
	cout << "Painting Door Steps" << endl;
	door_steps_map.shade(img);
	img->saveAsPGM("door_step.pgm");
	original.shade(img);
	for ( auto n : The_Doors ) {
		img->setPixel8U( n.x(), n.y() , 0);
	}
	img->saveAsPGM("The_Doors.pgm");
}

/** Main entry point
* @param argc Number of arguments.
* @param argv Name of .pgm file to open. Should be complete_map_project.pgm!
*/
int main(int argc, char** argv) {
	//(void)argc;
	if(argc!=2) {
		cerr << argc-1 << " parameters passed to program." << endl;
		return -1;
	}
	string filename(argv[1]);

	cout << "Loading image..." << endl;
	shared_ptr<Image> img(PPMLoader::load(filename));
	cout << "Image size: " << img->getWidth() << " x " << img->getHeight() << endl;

	pos_t robot_start = {ROBOT_START_X,ROBOT_START_Y};

	cout << "Robot test starting." << endl;
	cout << "Creating robot at " << robot_start << ".\n" << endl;
	robot myRobot(img,
				  ROBOT_DYNAMICS_RADIUS,
				  ROBOT_ARM_RADIUS,
				  ROBOT_SCANNER_RADIUS,
				  ROBOT_CUP_CAPACITY,
				  ROBOT_SPEED_PIX_PER_H,
				  robot_start);
	cout << "\nRobot created.\n\nSweeping floor.\n" << endl;
//	myRobot.sweep_floor(img,true);
	cout << "\nFloor sweeped.\n\nCollecting cups.\n" << endl;
	myRobot.cup_scan(img,false);
	cout << "\nCups have been collected.\n" << endl;
	cout << "Robot test done!" << endl;

	return 0;
}
