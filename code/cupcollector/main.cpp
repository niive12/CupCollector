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
	pixelshade_map original(img);
	//Run brushfire algorithm on reachable freespace:
	brushfire_map brush(img,start);
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
}
/** Main entry point
* @param argc Number of arguments. Should be 1!
* @param argv Name of .pgm file to open. Should be complete_map_project.pgm!
*/
int main(int argc, char** argv) {
	(void)argc;
	string filename(argv[1]);

	cout << "Loading image..." << endl;
	shared_ptr<Image> img(PPMLoader::load(filename));
	cout << "Image size: " << img->getWidth() << " x " << img->getHeight() << endl;

	pixelshade_map original(img );
	cout << "Giving start position for brushfire..." << endl;
	pos_t robot_start = {ROBOT_START_X,ROBOT_START_Y};

	cout << "Creating brushfire..." << endl;
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

	return 0;
}
