/** file */
#include "doordetector.h"
#include <cmath>

std::vector<pos_t> doorDetector::reduce_number_of_doors ( const std::vector<pos_t> &large_door_list, const brushfire_map &brushmap ) {
    /* Makes a map of where I have found a door.
     * If There is one door detected there is no need for another door to be on the list
     *
     * I do this by creating a map that tells if I have a door or I don't have a door.
     * If the area is clear I add the door to my final list of doors
     *
     * My output is a list of doors. Only one door per 18x18 area. (doorway size)
     */
    std::vector<pos_t> small_door_list;

    vector<vector<bool> > temp_map(brushmap.getWidth(),vector<bool>(brushmap.getHeight(),false));

    pos_t temp_pos;
    for (int j = 0; j < (int)large_door_list.size(); ++j ) { //check all doors in the list.
        temp_pos = large_door_list.at(j);
        bool clear = true;
        for( int x = -(WSPACE_DOORWAY_SIZE*2); x < (WSPACE_DOORWAY_SIZE*2); ++x ) {
	for ( int y = -(WSPACE_DOORWAY_SIZE*2); y < (WSPACE_DOORWAY_SIZE*2); ++y ) { //if there is no door in 18x18 then clear will remain true
	    if ( temp_map[temp_pos.x() + x][temp_pos.y() + y] ) {
	        clear = false;
	    }
	}
        }
        if ( clear ){ //update map and door list
	temp_map[temp_pos.x()][temp_pos.y()] = true;
	small_door_list.push_back( temp_pos );
        }
    }

    return small_door_list;
}

bool doorDetector::doorway_check ( pos_t pos, const brushfire_map &brushmap ){
    /* The way I check if a position is a doorway is to see if
     * there is a obstacle on both sides of the position.
     */

    auto door_distance = brushmap.const_coordVal( pos ) +1 ; // plus one if the doorway is an even number

    bool is_it_a_doorway = false;
    if ( ((int)pos.x() <= door_distance || (int)pos.x() >= (int)brushmap.getWidth()  -door_distance || //outside picture
          (int)pos.y() <= door_distance || (int)pos.y() >= (int)brushmap.getHeight() -door_distance ) ) {
          return false;
    }
    if ( WSPACE_IS_OBSTACLE(brushmap.const_coordVal( pos )) ) { //inside wall
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
        if ( WSPACE_IS_OBSTACLE(brushmap.const_coordVal( testing_pos_A ))
	 && WSPACE_IS_OBSTACLE(brushmap.const_coordVal( testing_pos_B ))) {
	is_it_a_doorway = true;
        }
    }

    /* Obstacle on both sides could also mean a hall.
     * To see if the position is next to open space I check
     * for an increase in the distance to the wall.
     * because of 8 point connectivity in brushfire I have to
     * check outside the door distance to see if there is an increase
     */

//    if ( is_it_a_doorway ) {
//        is_it_a_doorway = false;
//        pos_t test1_val;
//        pos_t test2_val;

//        int brushfire_val = brushmap.const_coordVal( pos );

//        for ( int i = 0; i < 8; ++i ) {
//	test1_val.x() = (int)(pos.x()) + connectivity[i][0] ;
//	test1_val.y() = (int)(pos.y()) + connectivity[i][1] ;
//	test2_val.x() = (int)(pos.x()) + connectivity[i][0]
//	        + (std::signbit(connectivity[i][0])?        //In some cases is this more likely
//		(WSPACE_DOOR_UNCERTAINTY):-(WSPACE_DOOR_UNCERTAINTY));
//	test2_val.y() = (int)(pos.y()) + connectivity[i][1]
//	        + (std::signbit(connectivity[i][1])?
//		(WSPACE_DOOR_UNCERTAINTY):-(WSPACE_DOOR_UNCERTAINTY));
//	if ( brushmap.const_coordVal( test1_val ) > brushfire_val ) {
//	    is_it_a_doorway = true;
//	} else if ( brushmap.const_coordVal( test2_val ) > brushfire_val ) {
//	    is_it_a_doorway = true;
//	}
//        }
//    }
    return is_it_a_doorway;
}


vector<pos_t> doorDetector::detect_doorways( const brushfire_map &brushmap ){
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
 std::cout << "halfway\n";
    for (int j = 0; j < (int)door_list_potential.size(); j++ ) {                   //append these coordinates to the list.
        if ( doorway_check( door_list_potential.at(j), brushmap ) ){
	door_list_true.push_back( door_list_potential.at(j) );
        }
    }
std::cout << "done\n";
//    door_list_true = reduce_number_of_doors ( door_list_true, brushmap );
    return move(door_list_true);
}
