/** @file
 * @author Lukas, Michael, Mikkel, Nikolaj & Mikael
 * @date 21-10-2014
 *
 * Group project.
*/
#pragma once

#include "libraries/Image.hpp"
#include "assignment.h"
#include <memory>
#include <vector>
#include <array>
#include <utility>
#include <set>
#include <queue>
#include <iostream>



using namespace rw::sensor;
using namespace std;

using coordIndexType = unsigned int;
struct pos_t : pair<coordIndexType, coordIndexType> {
    using x_type = first_type;
    using y_type = second_type;
    /** @brief to get reference to x-coordinate of pos_t coord, do coord.x(). */
    x_type &x() {return first;}
    y_type &y() {return second;}
    /** @brief to get const-reference to x-coordinate of pos_t coord, do coord.cx(). */
    x_type &cx() const {return const_cast<first_type&>(first);}
    y_type &cy() const {return const_cast<second_type&>(second);}

    /** Lvalue constructor, calls base class constructor */
    pos_t(const x_type &inx, const y_type &iny):pair::pair(inx,iny) {}
    /** Rvalue constructor, calls base class constructor */
    pos_t(x_type &&inx, y_type &&iny):pair::pair(move(inx),move(iny)) {}
    /** Default constructor, calls base class constructor */
    pos_t():pair::pair() {}
    /** Copy constructor, calls base class constructor */
    pos_t(const pair<x_type,y_type> &in):pair(in) {}
    /** Move constructor, calls base class constructor */
    pos_t(pair<x_type,y_type> &&in):pair(move(in)) {}

    pos_t make_pos(const x_type &inx, const y_type &iny) {return make_pair(inx,iny);}
    pos_t make_pos(x_type &&inx, y_type &&iny) {return make_pair(move(inx),move(iny));}
} ;


    /** @brief coordValType The value type for each pixel */
template<typename coordValType=long int>
class tekMap
{
public:

	/**
	 * @brief Convert map to pgm to visualize wave functionality.
	 * @param img. The image that I load and save to.
	 * @param fileName. Name of the output file.
	 */
	void convert_into_pgm( shared_ptr<Image> img, const std::string& fileName ){
		unsigned char val;
		for( coordIndexType x = 0; x < img->getWidth(); ++x){
			for ( coordIndexType y = 0; y < img->getHeight(); ++y){
//				val = ( myMap.at(x) ).at(y) % 255;
				val = (unsigned char) (const_coordVal(x, y) %255 );
				img->setPixel8U(x,y,val);
			}
		}
		img->saveAsPGM( fileName );
	}

    /** @brief mapType denotes the different map types... */
    typedef enum {UNINITIALIZED, WAVEFRONT, BRUSHFIRE, PIXELSHADE} mapType;

    /**
     * @brief map Constructor
     * @param img Pointer to the image
     * @param argMyType The type of map you want to construct
     * @param coords Pointer to set of offloading station coordinates.
     * @param reachableFreeSpace a coordinate inside the MAP, which the robot can reach.
     *
     * If reachableFreeSpace is supplied, the border-finding for the Brushfire
     * algorithm will be faster. Thus, it's only useful to supply it for Brushfire maps.
     */
    tekMap(shared_ptr< Image > img, const mapType argMyType = UNINITIALIZED,
        set<pos_t> coords = set< pos_t >() , const pos_t *reachableFreeSpace = nullptr)
        :myType(argMyType)
    {
        if( !img )
	cerr << "No image passed to map ctor" << endl;
        else if( myType != UNINITIALIZED )
        {
	if( myType == PIXELSHADE )
	{
	    myMap.clear();
	    myMap.resize( img->getWidth(), vector<coordValType>( img->getHeight() ));
	    //Just give pixel values to the map and be done with it.
	    for( size_t x = 0; x < img->getWidth(); ++x)
	    {
	        for( size_t y = 0; y < img->getHeight(); ++y)
		( myMap.at(x) ).at(y) = (coordValType)( img->getPixelValuei(x,y,0) );
	        ( myMap.at(x) ).shrink_to_fit();
	    }
	}
	else
	{
	    if( myType == BRUSHFIRE )
	    {
	        //Fill the coords set:
	        if(!reachableFreeSpace)
		coords = findObstacleBorders(img); //non-efficient
	        else
		coords = findObstacleBorders(img,*reachableFreeSpace); //faster
	    }

	    //Do wavefront on the coords set
	    // and fill the map with the resulting values:
	    wave(img,coords);
	}
        }
    }

    /** @brief getType returns the type of map...*/
    inline mapType getType() const
    { return myType; }

    /** @brief getCoordVal returns the coordinate value... */
    inline coordValType &const_coordVal(const pos_t::x_type &x, const pos_t::y_type &y) const
    { return const_cast<coordValType&>(((myMap.at(x)).at(y))); }
    inline coordValType &const_coordVal(pos_t::x_type &&x, pos_t::y_type &&y) const
    { return const_cast<coordValType&>(((myMap.at(move(x))).at(move(y)))); }
    inline coordValType &coordVal(const pos_t::x_type &x, const pos_t::y_type &y)
    { return ((myMap.at(x)).at(y)); }
    inline coordValType &coordVal(pos_t::x_type &&x, pos_t::y_type &&y)
    { return ((myMap.at(move(x))).at(move(y))); }
    inline coordValType &const_coordVal(const pos_t &ofThisCoord) const
    { return const_coordVal(ofThisCoord.first,ofThisCoord.second); }
    inline coordValType &const_coordVal(pos_t &&ofThisCoord) const
    { return const_coordVal(move(ofThisCoord.first),move(ofThisCoord.second)); }
    inline coordValType &coordVal(const pos_t &ofThisCoord)
    { return coordVal(ofThisCoord.first,ofThisCoord.second); }
    inline coordValType &coordVal(pos_t &&ofThisCoord)
    { return coordVal(move(ofThisCoord.first),move(ofThisCoord.second)); }

    inline size_t getHeight() const
    { return (myMap.at(0)).size(); }
    inline size_t getWidth() const
    { return myMap.size(); }

protected:
    /** @brief myType is this map's type*/
    mapType myType;

    /** @brief The map (dynamic 2D array) */
    vector<vector<coordValType> > myMap;

    /**
     * @brief isInImage Checks if the pixel is inside the image
     * @param img The image
     * @param x The x value...
     * @param y The y value...
     * @return true if pixel is inside image.
     */
    inline bool isInImage(const shared_ptr<Image> img, const int &x, const int &y) const
    {     return (((int)(img->getWidth()) > x) && ((int)(img->getHeight()) > y)); }

    /**
     * @brief findObstacleBorders returns all obstacle borders in image.
     * @param img The image to find obstacle borders in.
     * @return set of obstacle border coordinates.
     *    How it works:
     *      For all pixels in image:
     *          if pixel is an obstacle:
     *              if pixel has at least one freespace-neighbour:
     *                  return_value.insert(posType(x,y));
     *      return return_value.
     *
     *    So this is O(N).
     */
    virtual set< pos_t > findObstacleBorders(shared_ptr<Image> img) const
    {
        set<pos_t> resulting_coords;
        const array<array<int,2>,8> neighbours =
        {{  {-1,0}, /* W */ {1,0},  /* E */
	{0,-1}, /* N */ {0,1},  /* S */
	{-1,-1},/* NW */{1,-1}, /* NE */
	{1,1},  /* SE */{-1,1}  /* SW */
         }};

        for( int x = 0; x < (int)(img->getWidth()); ++x) {
	for( int y = 0; y < (int)(img->getHeight()); ++y) {
	    if( WSPACE_IS_OBSTACLE( img->getPixelValuei(x,y,0) ) ) {
	        for(auto n : neighbours) {
		//If neighbour is within image borders:
		if( isInImage( img, x+n.at(0), y+n.at(1) ) ) {
		    //if the neighbour is not an obstacle:
		    if( !WSPACE_IS_OBSTACLE( img->getPixelValuei(x+n.at(0),y+n.at(1),0) ) ) {
		        resulting_coords.insert(pos_t(x,y));
		        break;
		    }
		}
	        }
	    }
	}
        }

        return resulting_coords;
    }

    /**
     * @brief findObstacleBorders returns all RELEVANT obstacle borders in image.
     * @param img The image to find relevant obstacle borders in.
     * @param validFreeSpaceCoord A free space coordinate the robot can reach.
     * @return set of relevant obstacle border coordinates.
     *    How it works:
     *      Do
     *          Propagate wave from validFreeSpaceCoord.
     *              If wave hits an obstacle pixel:
     *                  return_value.insert(obstacle_pixel);
     *      until wave has visited all freespace.
     *      return return_value.
     *
     *    So this is O(N) but performs better than the other findObstacleBorders.
     */
    virtual inline set< pos_t > findObstacleBorders(shared_ptr<Image> img, const pos_t &validFreeSpaceCoord) const
    { return findCoords(img,validFreeSpaceCoord,true); }

    /**
     * @brief wave I have no idea if this works. But if it does, it's awesome.
     * @param img Image pointer
     * @param goals set of goal coordinates
     */
    virtual void wave(shared_ptr<Image> img, const set<pos_t> &goals)
    {
        const array<array<int,2>,8> neighbours =
        {{  {-1,0}, /* W */ {1,0},  /* E */
	{0,-1}, /* N */ {0,1},  /* S */
	{-1,-1},/* NW */{1,-1}, /* NE */
	{1,1},  /* SE */{-1,1}  /* SW */
         }};

        myMap.clear();
        myMap.resize( img->getWidth(), vector<coordValType>( img->getHeight(),WAVE_VAL_UNV ));

        queue<pos_t> q; //FIFO
        for(auto i : goals)
        {
	(myMap.at(i.first)).at(i.second) = WAVE_VAL_GOAL;
	q.push(i);
        }

        while(!q.empty())
        {
	pos_t cur = q.front();
	q.pop();
	for(auto n : neighbours) {
	    //If neighbour is within image borders:
	    if( isInImage( img, cur.first+n.at(0), cur.second+n.at(1) ) ) {
	        //if the neighbour is an unvisited non-obstacle:
	        if( ( !WSPACE_IS_OBSTACLE( img->getPixelValuei(cur.first+n.at(0),cur.second+n.at(1),0) ) )
		    && (
		        (  myMap.at(cur.first+n.at(0)).at(cur.second+n.at(1)) == WAVE_VAL_UNV ) //unvisited
		        || ( ( myMap.at(cur.first+n.at(0)).at(cur.second+n.at(1)) )
			 > ( myMap.at(cur.first).at(cur.second) )
			 )
		        )
		    )
	        {
		//increment:
		myMap.at(cur.first+n.at(0)).at(cur.second+n.at(1))
		        = myMap.at(cur.first).at(cur.second) +1;
		//Add it to the wavefront
		q.push(pos_t(cur.first+n.at(0),cur.second+n.at(1)));
	        }
	    }
	}
        }
    }

    /**
     * @brief findCoords Finds workspace (freespace) coordinates or relevant borders.
     * @param img Image pointer
     * @param withinFreeSpace A coordinate within the freespace, eg. a valid coordinate.
     * @param borders If true, only coords bordering the freespace are returned,
     *                if false, only coords within the freespace are returned.
     * @return Coords bordering or within the freespace, depending on the borders parameter.
     */
    virtual set< pos_t > findCoords(shared_ptr<Image> img, const pos_t &withinFreeSpace, const bool borders=true) const
    {
        set<pos_t> resulting_coords;

        if(!isInImage(img,withinFreeSpace.first,withinFreeSpace.second))
	cerr << "coord given to findCoords is not in image." << endl;
        else {
	const array<array<int,2>,8> neighbours =
	{{  {-1,0}, /* W */ {1,0},  /* E */
	    {0,-1}, /* N */ {0,1},  /* S */
	    {-1,-1},/* NW */{1,-1}, /* NE */
	    {1,1},  /* SE */{-1,1}  /* SW */
	 }};

	vector<vector<bool> > visited(img->getWidth(),vector<bool>(img->getHeight(),false));
	queue<pos_t> q; //FIFO
	q.push(withinFreeSpace);

	while(!q.empty())
	{
	    pos_t cur = q.front();
	    q.pop();

	    if(!borders)
	        resulting_coords.insert(cur);

	    for(auto n : neighbours) {
	        //If neighbour is within image borders:
	        if( isInImage( img, cur.first+n.at(0), cur.second+n.at(1) ) ) {
		//if the neighbour is an unvisited non-obstacle:
		if( !WSPACE_IS_OBSTACLE( img->getPixelValuei(cur.first+n.at(0),cur.second+n.at(1),0) )
		        && !( (visited.at(cur.first+n.at(0))).at(cur.second+n.at(1)) ) ) {
		    //Add it to the wavefront
		    (visited.at(cur.first+n.at(0))).at(cur.second+n.at(1)) = true;
		    q.push(pos_t(cur.first+n.at(0),cur.second+n.at(1)));
		}
		else if(borders){
		    //If the neighbour WAS an obstacle, it must have been a border.
		    resulting_coords.insert(cur);
		}
	        }
	    }
	}
        }

        return move(resulting_coords);
    }

};
using brushfire_map = tekMap<unsigned char>;
using wavefront_map = tekMap<long unsigned int>;
using pixelshade_map = tekMap<unsigned char>;
