/** file */
#include "doordetector.h"
#include <cmath>

/**
 * @brief doorDetector::doorway_check
 * @param img                   An image of the entire map to check for obstacles
 * @param pos                   A position to check if it is a doorway
 * @return		        True or False
 * The way I check if a position is a doorway is to see if
 * there is a obstacle on both sides of the position.
 */
bool doorDetector::doorway_check ( shared_ptr<Image> img, pos_t pos, const brushfire_map &brushmap){

    auto door_distance = brushmap.const_coordVal( pos ) +1 ; // plus one if the doorway is an even number

    bool is_it_a_doorway = false;
    if ( ((int)pos.x() <= door_distance || (int)pos.x() >= (int)brushmap.getWidth()  -door_distance || //outside picture
          (int)pos.y() <= door_distance || (int)pos.y() >= (int)brushmap.getHeight() -door_distance ) ) {
	    std::cout << "| " << door_distance << " |\n";
	    std::cout << "| " << brushmap.getWidth() << " x " << brushmap.getHeight() << " |\n";
	    std::cout << "( " << pos.x() << " , " << pos.y() << " )\n";
          return false;
    }
    if ( WSPACE_IS_OBSTACLE(img->getPixelValuei( pos.x(), pos.y() , 0 ) ) ) { //inside wall
        return false;
    }

    pos_t testing_pos_A;
    pos_t testing_pos_B;

    const array<array<int,2>,8> connectivity =
    {{   {            0, door_distance}, /*S*/  {            0,-door_distance} /*N*/
        ,{ door_distance,            0}, /*E */ {-door_distance,            0} /*W */
        ,{ door_distance,-door_distance}, /*SE*/ {-door_distance, door_distance} /*NW*/
        ,{-door_distance,-door_distance}, /*SW*/ { door_distance, door_distance} /*NE*/
     }};

    for ( int i = 0; i < 4; i += 2 ) {
        testing_pos_A.x() = (long int)(pos.x()) + connectivity[i][0];
        testing_pos_A.y() = (long int)(pos.y()) + connectivity[i][1];
        testing_pos_B.x() = (long int)(pos.x()) + connectivity[i+1][0];
        testing_pos_B.y() = (long int)(pos.y()) + connectivity[i+1][1];
        if ( WSPACE_IS_OBSTACLE(img->getPixelValuei( pos.x(), pos.y(), 0 ) ) &&
	   WSPACE_IS_OBSTACLE(img->getPixelValuei( pos.x(), pos.y(), 0) )) {
	is_it_a_doorway = true;
        }
    }
    return is_it_a_doorway;
}

/**
 * @brief doorDetector::detect_doorways
 * @param img               Image of the entire map
 * @param brushmap     Brushfire map is calculated elsewhere
 * @return                       A list of doors
 *
 * Detects doorways by looking at brushfire values.
 * If two waves collide the corner will form a T.
 * This T will have the distance to the center of the doorframe
 */

vector<pos_t> doorDetector::detect_doorways( shared_ptr<Image> img, const brushfire_map &brushmap ){
    std::vector<pos_t> door_list_potential; //list of doors.
    std::vector<pos_t> door_list_true; //list of doors.
    const std::array<std::array<int,2>,4> neighbours = {{
		    {-1,0}, /* W */ {1,0},  /* E */
		    {0,-1}, /* N */ {0,1},  /* S */ }};
    std::vector<std::array<int,2>> matches;

    brushfire_map::myValType val;
    for(coordIndexType x=1;x<(coordIndexType)(brushmap.getWidth())-1;++x){ //search entire configuration space
        for(coordIndexType y=1;y<(coordIndexType)(brushmap.getHeight())-1;++y){
	        val = brushmap.const_coordVal( x, y );
	        for(auto n : neighbours) {
		        if ( brushmap.const_coordVal( x + n.at(0), y + n.at(1) ) != val ) {
			        matches.push_back( n );
		        }
	        }
	        if ( matches.size() == 1 ) {
		        pos_t possible_door(matches.back().at(0)*(-1*val) + x, matches.back().at(1)*(-1*val) + y);
		        if ( !((int)possible_door.x() < 0 || (int)possible_door.x() > (int)brushmap.getWidth() || //outside picture
			  (int)possible_door.y() < 0 || (int)possible_door.y() > (int)brushmap.getHeight() ) ) {

		        door_list_potential.push_back( possible_door  );
		        }
	        }
	        matches.clear();
        }
    }
    std::cout << "possible doors = " << door_list_potential.size() << std::endl;
    for (int j = 0; j < (int)door_list_potential.size(); j++ ) {                   //append these coordinates to the list.
        if ( doorway_check( img, door_list_potential.at(j), brushmap ) ){
	door_list_true.push_back( door_list_potential.at(j) );
        }
    }
    std::cout << "after detection = " << door_list_true.size() << std::endl;
    return move(door_list_true);
}

/**
 * @brief doorDetector::door_step
 * @param img                     The image class is used to detect obstacles
 * @param brushmap           The brushmap is used to see distance to door frame
 * @param the_doors           A list of doors found by detect_doorways function.
 * @return door_step_map  The map with the door steps
 *
 * Go through every door in list
 * Check direction by checking if there is an obstacle on both sides of door
 * paint pixel black from door to obstacle in door direction
 * return a map with door steps painted black
 */
pixelshade_map doorDetector::door_step(shared_ptr<Image> img, brushfire_map &brushmap, std::vector<pos_t> the_doors){
	pixelshade_map door_step_map(img);

	brushfire_map::myValType val;

	pos_t test_A, test_B;

	pos_t relative;

	const std::array<std::array<int,2>,4> directions = {{
			{-1,0}, /* W */ {1,0},  /* E */
			{0,-1}, /* N */ {0,1},  /* S */ }};

	for ( auto door : the_doors ) {
		val = brushmap.const_coordVal( door );
		for ( int n = 0; n < 4; n += 2 ) {
			test_A.x() = (long int)(door.x()) + (directions[n][0] * (val +1 ));
			test_A.y() = (long int)(door.y()) + (directions[n][1] * (val +1));
			test_B.x() = (long int)(door.x()) + (directions[n+1][0] *(val + 1));
			test_B.y() = (long int)(door.y()) + (directions[n+1][1] * (val + 1));

			if ( brushmap.const_coordVal( test_A ) == WSPACE_OBSTACLE &&
			      brushmap.const_coordVal( test_B ) == WSPACE_OBSTACLE ) {
				for ( int dir = 0; dir < 2; ++dir){
					for ( int i = 0 ; i < val;++i) {
						relative.x() = door.x() + (directions[n+dir][0] * i);
						relative.y() = door.y() + (directions[n+dir][1] * i);
						door_step_map.coordVal( relative ) = WSPACE_OBSTACLE;
					}
				}
			}
		}
	}
	return move( door_step_map );
}
