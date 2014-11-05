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
#include "doordetector/doordetector.h"
#include "scanner/scanner.h"

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
    //pixelshade_map original(img,pixelshade_map::PIXELSHADE);
    pixelshadeMap original(img);
    //Run brushfire algorithm on reachable freespace:
    //brushfire_map brush(img,brushfire_map::BRUSHFIRE,list< pos_t >(),&start);
    //brushfire_map brush(img,brushfire_map::BRUSHFIRE);
    brushfireMap brush(img,start);
    //Paint the brushfire values onto the canvas:
    brush.shade(canvas);
    //Save the brushfire painting:
    canvas->saveAsPGM("brushfire.pgm");
    //Run wavefront algorithm on reachable freespace, goals being the Offloading stations:
    //wavefront_map wave(img,wavefront_map::WAVEFRONT,wavefront_map::getOffloadingStations(img));
    wavefrontMap wave(img,wavefrontMap::getOffloadingStations(img));
    //Paint the wavefront values onto the canvas:
    wave.shade(canvas);
    //Save the wavefront painting:
    canvas->saveAsPGM("wavefront.pgm");
    //Turn the painted canvas back to the original:
    original.shade(canvas);
    //Save the original painting:
    canvas->saveAsPGM("original.pgm");

    cout << "dijkstra" << endl;
    dijkstraMap d(img,dijkstraMap::getOffloadingStations(img));
    cout << "dijkstra done" << endl;
    d.shade(canvas);
    canvas->saveAsPGM("dijkstra.pgm");

}

/** Main entry point
 * @param argc Number of arguments. Should be 1!
 * @param argv Name of .pgm file to open. Should be complete_map_project.pgm!
 */
int main(int argc, char** argv) {
    (void)argc;
    string filename(argv[1]);
    pos_t hej = {ROBOT_START_X,ROBOT_START_Y};

    cout << "Loading image..." << endl;
    shared_ptr<Image> img(PPMLoader::load(filename));
    cout << "Image size: " << img->getWidth()
         << " x " << img->getHeight() << endl;

    //pixelshade_map original(img,pixelshade_map::PIXELSHADE);
    pixelshadeMap original(img);

    //brushfire_map brush(img,brushfire_map::BRUSHFIRE,list< pos_t >(),&hej);
    brushfireMap brush(img,hej);
    doorDetector mydetective;
    cout << "Finding The Doors " << endl;
    vector<pos_t> The_Doors = mydetective.detect_doorways(brush);
    cout << "Painting The Doors" << endl;
    for(auto i : The_Doors)
        img->setPixel8U(i.x(),i.y(),5);
    cout << "Saving The Doors as The_Doors.pgm..." << endl;
    img->saveAsPGM("The_Doors.pgm");

    original.shade(img);

    pos_t two_cups = {1320,1257};
    scanner cs;
    cout << "\nNumber of cups at ( " << two_cups.cx() << " , "
         << two_cups.cy() << " ): " << cs.scan(two_cups,original)
         << ", say again: " << cs.scan(two_cups,original) << endl;

    testTekMapConstructors(filename);

    return 0;
}


void shared_ptrs_for_beginners()
{
	//anything represents any class (here, it's just int)
	typedef int anything;

	//How people did it before C++11:
	//Let's hope there aren't thrown any exceptions in anything's constructor!
	anything *theOldWay = new anything();
	*theOldWay = 5;
	delete theOldWay;
	// Uncomment to crash shit:
	// *theOldWay = 6;

	//The smart way (class name used only once):
	auto theSmartWay = make_shared<anything>(/* anything constructor args*/);
	*theSmartWay = 5;

	//The other smart way (class name used twice here):
	shared_ptr<anything> theOtherSmartWay(new anything());
	*theOtherSmartWay = 5;

	// If you have a function like this:
	// anything * foo() { return new anything(); };
	//
	// then theOtherSmartWay would be:
	// shared_ptr<anything> theOtherSmartWay(foo());
	//
	// Note that PPMLoader::load("filename") returns an Image pointer
	// allocated by new, so it can be used in the above manner.

	cout << "Come on guys!" << endl;
}
