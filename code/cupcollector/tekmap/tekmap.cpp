/** @file */
#include "tekmap.h"
#include "assignment.h"
#include <queue>
#include <set>
#include <array>
#include <memory>
#include <iostream>

inline bool tekMap::isInImage(const shared_ptr<Image> img, const int &x, const int &y) const {
    return (((int)(img->getWidth()) > x) && ((int)(img->getHeight()) > y));
}

inline tekMap::mapType tekMap::getType() const
{ return myType; }

inline tekMap::coordValType &tekMap::cgetCoordVal(const pos_t &ofThisCoord) const {
    return const_cast<coordValType&>(((myMap.at(ofThisCoord.first)).at(ofThisCoord.second)));
}
inline tekMap::coordValType &tekMap::getCoordVal(const pos_t &ofThisCoord) {
    return ((myMap.at(ofThisCoord.first)).at(ofThisCoord.second));
}


tekMap::tekMap(shared_ptr< Image > img, const mapType argMyType, set<pos_t> coords, const pos_t *reachableFreeSpace)
    :myType{ argMyType }
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

set<tekMap::pos_t> tekMap::findObstacleBorders(shared_ptr<Image> img) const
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

void tekMap::wave(shared_ptr<Image> img, const set<pos_t> &goals)
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



set<tekMap::pos_t> tekMap::findCoords(shared_ptr<Image> img, const pos_t &withinFreeSpace, const bool borders) const
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

    return resulting_coords;
}

inline set<tekMap::pos_t> tekMap::findObstacleBorders(shared_ptr<Image> img, const pos_t &validFreeSpaceCoord) const
{
    return findCoords(img,validFreeSpaceCoord,true);
}
