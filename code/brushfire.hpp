#pragma once
#include "loader/PPMLoader.hpp"
#include "image/Image.hpp"
#include "settings/macros.hpp"
#include <string>

#define DOORWAY_SIZE        9
#define DOOR_UNCERTAINTY    2
#define COFFEE_CUP          150

using namespace rw::sensor;

template<class map_type> class Configuration_space {

private:
	map_type** data;
	void convert_into_pgm( Image* img, const std::string& fileName );
	
	int width; 
	int height;
	
	std::vector<position_t> wave_check ( position_t pos, int previous_distance ); //checks surrounding pixels
	std::vector<position_t> reduce_number_of_doors ( std::vector<position_t> large_door_list );
	bool doorway_check ( position_t pos ); 
	
	void wave_func( std::vector<position_t> search_list, map_type max_estimate = -1 );
	
public:
	Configuration_space( Image* img );
	
	void set_value( position_t pos, unsigned int value );
	unsigned int get_value( position_t pos );
	
	std::vector<position_t> brushfire_check ( position_t pos, unsigned char previous_distance );

	std::vector<position_t> detect_doorways( );
	
	void brushfire( );
};


template<class map_type> std::vector<position_t> Configuration_space<map_type>::reduce_number_of_doors ( std::vector<position_t> large_door_list ) {
	/* Makes a map of where I have found a door.
	 * If There is one door detected there is no need for another door to be on the list
	 * 
	 * I do this by creating a map that tells if I have a door or I don't have a door.
	 * If the area is clear I add the door to my final list of doors
	 * 
	 * My output is a list of doors. Only one door per 18x18 area. (doorway size)
	 */
	
	std::vector<position_t> small_door_list;
	
	
	bool** temp_map; //create a map
	temp_map = new bool* [width]; 
	for(int w=0;w<width;w++){
		temp_map[w]=new bool[height];
	}
	
	for( int x = 0; x < width; x++ ) { //set every value to false
		for ( int y = 0; y < height; y++ ) {
			temp_map[x][y] = false;
		}
	}
	position_t temp_pos;
	for (int j = 0; j < large_door_list.size(); j++ ) { //check all doors in the list.
		temp_pos = large_door_list.at(j);
		bool clear = true;
		for( int x = -(DOORWAY_SIZE*2); x < (DOORWAY_SIZE*2); x++ ) {
			for ( int y = -(DOORWAY_SIZE*2); y < (DOORWAY_SIZE*2); y++ ) { //if there is no door in 18x18 then clear will remain true
				if ( temp_map[temp_pos.x + x][temp_pos.y + y] ) {
					clear = false;
				}
			}
		}
		if ( clear ){ //update map and door list
			temp_map[temp_pos.x][temp_pos.y] = true;
			small_door_list.push_back( temp_pos );
		}
	}
	
	for(size_t i = 0; i < width; i++){ //cleanup
		delete[] temp_map[i];
	}
	delete[] temp_map;
	
	return small_door_list;
}

template<class map_type> bool Configuration_space<map_type>::doorway_check ( position_t pos ){ 
	/* The way I check if a position is a doorway is to see if 
	 * there is a obstacle on both sides of the position.
	 */
	
	bool is_it_a_doorway = false;
	if ( (pos.x < 2*DOORWAY_SIZE || pos.x > width  -2*DOORWAY_SIZE || //outside picture
	      pos.x < 2*DOORWAY_SIZE || pos.y > height -2*DOORWAY_SIZE ) ) {
		  return false;
	}
	if ( get_value( pos ) == WAVE_OBSTACLE ) { //inside wall 
		return false;
	}
	position_t testing_pos_A;
	position_t testing_pos_B;
	
	position_t connectivity[8] = {  {            0, DOORWAY_SIZE}, /*S*/  {            0,-DOORWAY_SIZE} /*N*/    
	                               ,{ DOORWAY_SIZE,            0}, /*E */ {-DOORWAY_SIZE,            0} /*W */
	                               ,{ DOORWAY_SIZE,-DOORWAY_SIZE}, /*SE*/ {-DOORWAY_SIZE, DOORWAY_SIZE} /*NW*/
	                               ,{-DOORWAY_SIZE,-DOORWAY_SIZE}, /*SW*/ { DOORWAY_SIZE, DOORWAY_SIZE} /*NE*/ };
	
	for ( int i = 0; i < 4; i = i + 2 ) { 
		testing_pos_A.x = pos.x + connectivity[i].x;
		testing_pos_A.y = pos.y + connectivity[i].y;
		testing_pos_B.x = pos.x + connectivity[i+1].x;
		testing_pos_B.y = pos.y + connectivity[i+1].y;
		if ( get_value( testing_pos_A ) == WAVE_OBSTACLE && get_value( testing_pos_B ) == WAVE_OBSTACLE ) {
			is_it_a_doorway = true;
		}
	}
	
	/* Obstacle on both sides could also mean a hall.
	 * To see if the position is next to open space I check 
	 * for an increase in the distance to the wall.
	 * because of 8 point connectivity in brushfire I have to 
	 * check outside the door distance to see if there is an increase
	 */
	
	if ( is_it_a_doorway ) { 
		is_it_a_doorway = false;
		position_t test1_val;
		position_t test2_val;
		
		int brushfire_val = get_value ( pos );
		
		for ( int i = 0; i < 8; i++ ) {
			test1_val.x = pos.x + connectivity[i].x ;
			test1_val.y = pos.y + connectivity[i].y ;
			test2_val.x = pos.x + connectivity[i].x + DOOR_UNCERTAINTY; //In some cases is this more likely
			test2_val.y = pos.y + connectivity[i].y + DOOR_UNCERTAINTY;
			if ( get_value ( test1_val ) > brushfire_val ) {
				is_it_a_doorway = true;
			} else if ( get_value ( test2_val ) > brushfire_val ) {
				is_it_a_doorway = true;
			}
		}
	}
	return is_it_a_doorway;
}

template<class map_type> std::vector<position_t> Configuration_space<map_type>::detect_doorways( ){
	std::vector<position_t> door_list_potential; //list of doors.
	std::vector<position_t> door_list_true; //list of doors.

	position_t i;
	for(i.x=0;i.x<width;i.x++){ //search entire configuration space
		for(i.y=0;i.y<height;i.y++){
			if ( get_value( i ) >= DOORWAY_SIZE +2 - DOOR_UNCERTAINTY && //Mark every node that could be a doorway
			     get_value( i ) <= DOORWAY_SIZE +2 + DOOR_UNCERTAINTY ) {				
				door_list_potential.push_back( i );
			}
		}
	}
	i.x = 1;
	i.y = 1;
	for (int j = 0; j < door_list_potential.size(); j++ ) {                   //append these coordinates to the list.
		if ( doorway_check( door_list_potential.at(j) ) ){
			door_list_true.push_back( door_list_potential.at(j) );
		}
	}
	
	door_list_true = reduce_number_of_doors ( door_list_true );
	return door_list_true;
}

template<class map_type> void Configuration_space<map_type>::brushfire ( ){
	
	std::vector<position_t> search_list; //list of pixels to search for.
	
	position_t i;
	for(i.x=0;i.x<width;i.x++){ //get original obstacles.
		for(i.y=0;i.y<height;i.y++){
			if ( get_value( i ) == WAVE_OBSTACLE ) {
				search_list.push_back( i );
			}
		}
	}
	
	wave_func ( search_list, 170 );
}
template<class map_type> void Configuration_space<map_type>::wave_func( std::vector<position_t> search_list, map_type max_estimate ) {
	map_type distance = 2; //OBSTACLES is defined as 1
	 
	bool image_complete = false;
	position_t picture; 
	std::vector<position_t> surrounding_points;
	std::vector<position_t> checked_list; //list of pixels checked with the current distance.
	while ( !image_complete ) {
		image_complete = true; 
		while ( search_list.size() > 0 ) {
			image_complete = false; //prevent the loop to stop when there is more pixels to search for
			picture = search_list.back(); 
			search_list.pop_back();       
			surrounding_points = brushfire_check( picture, distance ); //this gives the surrounding coordinates 
			while ( surrounding_points.size() > 0 ) {                   //append these coordinates to the list.
				checked_list.push_back( surrounding_points.back() );
				surrounding_points.pop_back();
			}
		}
		search_list = checked_list; 
		checked_list.clear();
		distance++;
		if ( distance > max_estimate ) { 
			break;
		}
	}
}

template<class map_type> std::vector<position_t> Configuration_space<map_type>::brushfire_check ( position_t pos, unsigned char previous_distance ){ 
	std::vector <position_t> checked_positions;
	//check 8 surrounding pixels
	unsigned char distance = previous_distance + 1;
	position_t testing_pos;
	
	for ( int x = -1; x <= 1; x++ ) {
		for ( int y = -1; y <= 1; y++ ) {
			testing_pos.x = pos.x + x;
			testing_pos.y = pos.y + y;
			if ( (testing_pos.x >= 0 && testing_pos.x < width && //within picture
			      testing_pos.y >= 0 && testing_pos.y < height) ) {
				if ( get_value( testing_pos ) == WAVE_FREE ) {
					set_value(testing_pos, distance);
					checked_positions.push_back( testing_pos );
				}
			}
		}
	}
	return checked_positions;
}

template<class map_type> Configuration_space<map_type>::Configuration_space ( Image* img ) {
	width = img->getWidth();
	height = img->getHeight();
	
	data = new map_type* [width];
	for(int w=0;w<width;w++){
		data[w]=new map_type[height];
	}
	
	map_type val;
	position_t i;
	for(i.x=0;i.x<width;i.x++){
		for(i.y=0;i.y<height;i.y++){
			val = img->getPixelValuei( i.x, i.y, CHANNEL );

			if ( val == OBSTACLE ) {
				val = WAVE_OBSTACLE;
			} else {
				val = WAVE_FREE;
			}
			data[i.x][i.y] = val;
		}
	}
};

template<class map_type> void Configuration_space<map_type>::set_value( position_t pos, unsigned int value ){
	data[pos.x][pos.y] = value;
};

template<class map_type> unsigned int Configuration_space<map_type>::get_value( position_t pos ){
	return data[pos.x][pos.y];
};

template<class map_type> void Configuration_space<map_type>::convert_into_pgm( Image* img, const std::string& fileName ){
	Image* temp_img = img;
	for( int x = 0; x < width; x++){
		for ( int y = 0; y < height; y++){
			int val = data[x][y];
			if ( val == WAVE_OBSTACLE ) {
				val = OBSTACLE;
			} else if ( val != WAVE_FREE ) {
				val = PATH_COLOR;
			} else {
				val = FREE_SPACE;
			}
			temp_img->setPixel8U(x,y,val);
		}
	}
	temp_img->saveAsPGM( fileName );
};
