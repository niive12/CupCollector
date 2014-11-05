/** @file
 * @author Lukas, Michael, Mikkel, Nikolaj & Mikael
 * @date 21-10-2014
 *
 * Group project.
*/
#pragma once

#include "../libraries/Image.hpp"
#include "../assignment.h"
#include <memory>
#include <vector>
#include <array>
#include <utility>
#include <set>
#include <list>
#include <queue>
#include <iostream>
#include <limits>


using namespace rw::sensor;
using namespace std;

using coordIndexType = long int;
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
    pos_t(const x_type &inx, const y_type &iny):pair(inx,iny) {}
    /** Rvalue constructor, calls base class constructor */
    pos_t(x_type &&inx, y_type &&iny):pair(move(inx),move(iny)) {}
    /** Default constructor, calls base class constructor */
    pos_t():pair() {}
    /** Copy constructor, calls base class constructor */
    pos_t(const pair<x_type,y_type> &in):pair(in) {}
    /** Move constructor, calls base class constructor */
    pos_t(pair<x_type,y_type> &&in):pair(move(in)) {}

    pos_t make_pos(const x_type &inx, const y_type &iny) {return make_pair(inx,iny);}
    pos_t make_pos(x_type &&inx, y_type &&iny) {return make_pair(move(inx),move(iny));}

    friend const pos_t operator+(const pos_t& a, const pos_t& b)
    { return move(pos_t(a.cx()+b.cx(),a.cy()+b.cy())); }
    friend const pos_t operator+(pos_t&& a, pos_t&& b)
    { return move(pos_t(a.cx()+b.cx(),a.cy()+b.cy())); }
    friend const pos_t operator-(const pos_t& a, const pos_t& b)
    { return move(pos_t(a.cx()-b.cx(),a.cy()-b.cy())); }
    friend const pos_t operator-(pos_t&& a, pos_t&& b)
    { return move(pos_t(a.cx()-b.cx(),a.cy()-b.cy())); }

} ;

/** @brief coordValType The value type for each pixel */
template<typename coordValType=long int>
class tekMap
{
public:
    using myValType = coordValType;
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

    tekMap(shared_ptr<Image> img)
    {
        if( !img )
            cerr << "No image passed to map ctor" << endl;
    }


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

    void shade(shared_ptr<Image> img)
    {
        coordValType mymin=numeric_limits<coordValType>::max();
        coordValType mymax=numeric_limits<coordValType>::min();
        for(unsigned int x=0; x<getWidth(); ++x)
            for(unsigned int y=0; y<getHeight(); ++y) {
                if(myMap[x][y]>mymax && myMap[x][y]!=WAVE_VAL_UNV)
                    mymax=myMap[x][y];
                if(myMap[x][y]<mymin)
                    mymin=myMap[x][y];
            }
        for(unsigned int x=0; x<getWidth(); ++x)
            for(unsigned int y=0; y<getHeight(); ++y) {
                img->setPixel8U(x,y,
                                (unsigned char)(((myMap[x][y]==WAVE_VAL_UNV?(mymax-mymin):myMap[x][y])*255)/((mymax-mymin)))
                                );
            }
    }

    static list<pos_t> getOffloadingStations(const shared_ptr<Image> img) {
        list<pos_t> result;
        for(coordIndexType x=0; x<coordIndexType(img->getWidth()); ++x)
            for(coordIndexType y=0; y<coordIndexType(img->getHeight()); ++y)
                if(WSPACE_IS_OL_STATION(img->getPixelValuei(x,y,0)))
                    result.emplace_back(x,y);
        return move(result);
    }

protected:

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
    {
        return ((((int)(img->getWidth()) > x) && ((int)(img->getHeight()) > y))
                &&((x>0) && (y>0)) );
    }

    /**
     * @brief wave Stores all wavefront algorithm values in myMap.
     * @param img Image pointer
     * @param goals set of goal coordinates
     */
    virtual void wave(shared_ptr<Image> img, const list<pos_t> &goals)
    {
        const array<array<int,2>,8> neighbours =
        {{  {-1,0}, /* W */ {1,0},  /* E */
            {0,-1}, /* N */ {0,1},  /* S */
            {-1,-1},/* NW */{1,-1}, /* NE */
            {1,1},  /* SE */{-1,1}  /* SW */
         }};
        myMap.clear();
        myMap.resize( img->getWidth(), vector<coordValType>( img->getHeight() , WAVE_VAL_UNV));
        vector<vector<bool> > visited(img->getWidth(),vector<bool>(img->getHeight(),false));
        queue<pos_t> q; //FIFO
        for(auto i : goals) {
            (myMap[i.first])[i.second] = WAVE_VAL_GOAL;
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
                            && ( !( (visited[cur.first+n.at(0)])[cur.second+n.at(1)] ) )
                            && ((myMap[cur.first+n.at(0)])[cur.second+n.at(1)] != WAVE_VAL_GOAL)
                            )
                    {
                        //increment:
                        (myMap[cur.first+n.at(0)])[cur.second+n.at(1)]
                                = (myMap[cur.first])[cur.second] +1;
                        (visited[cur.first+n.at(0)])[cur.second+n.at(1)]=true;

                        //Add it to the wavefront
                        q.emplace(cur.first+n.at(0),cur.second+n.at(1));
                    }
                }
            }
            (visited[cur.first])[cur.second]=true;
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
    virtual list< pos_t > findCoords(shared_ptr<Image> img, const pos_t &withinFreeSpace, const bool borders=true) const
    {
        list<pos_t> resulting_coords;

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

                if(!borders) //We know it's unvisited.
                    //resulting_coords.insert(cur);
                    resulting_coords.push_back(cur);

                for(auto n : neighbours) {
                    //If neighbour is within image borders:
                    if( isInImage( img, cur.first+n.at(0), cur.second+n.at(1) ) ) {
                        //if the neighbour is an unvisited non-obstacle:
                        if( (!WSPACE_IS_OBSTACLE( img->getPixelValuei(cur.first+n.at(0),cur.second+n.at(1),0) ))
                                && (!( (visited[cur.first+n.at(0)])[cur.second+n.at(1)] ) ) ) {
                            //Add it to the wavefront
                            q.push(pos_t(cur.first+n.at(0),cur.second+n.at(1)));
                            (visited[cur.first+n.at(0)])[cur.second+n.at(1)]=true;
                        }
                        if(borders&&WSPACE_IS_OBSTACLE( img->getPixelValuei(cur.first+n.at(0),cur.second+n.at(1),0) )){
                            //If the neighbour WAS an obstacle, it must have been a border.
                            if(!((visited[cur.first+n.at(0)])[cur.second+n.at(1)])) {
                                //resulting_coords.emplace(cur.first+n.at(0),cur.second+n.at(1));
                                resulting_coords.emplace_back(cur.first+n.at(0),cur.second+n.at(1));
                                (visited[cur.first+n.at(0)])[cur.second+n.at(1)]=true;
                            }
                        }
                    }
                }
                (visited[cur.first])[cur.second] = true;
            }
        }

        return move(resulting_coords);
    }

};

class brushfireMap : public tekMap<unsigned char>
{
protected:
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
    virtual list< pos_t > findObstacleBorders(shared_ptr<Image> img) const
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

        return move(list<pos_t>(resulting_coords.begin(),resulting_coords.end()));
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
    virtual inline list< pos_t > findObstacleBorders(shared_ptr<Image> img, const pos_t &validFreeSpaceCoord) const
    { return move(findCoords(img,validFreeSpaceCoord,true)); }

public:
    brushfireMap(shared_ptr< Image > img)
        :tekMap(img)
    {
        list<pos_t> coords;
        //Fill the coords set:
        coords = findObstacleBorders(img); //non-efficient
        wave(img,coords);
    }

    brushfireMap(shared_ptr< Image > img,  const pos_t &reachableFreeSpace)
        :tekMap(img)
    {
        list<pos_t> coords;
        //Fill the coords set:
        coords = findObstacleBorders(img,reachableFreeSpace); //faster
        wave(img,coords);
    }
};

class wavefrontMap : public tekMap<long unsigned int>
{
public:
    wavefrontMap(shared_ptr< Image > img, const list<pos_t> &coords = list< pos_t >())
        :tekMap(img)
    {
        if(coords.empty())
            cerr << "No coords passed to wavefrontMap constructor!" << endl;
        wave(img,coords);
    }

};

class pixelshadeMap : public tekMap<unsigned char>
{
public:
    pixelshadeMap(shared_ptr< Image > img)
        :tekMap(img)
    {
        myMap.clear();
        myMap.resize( img->getWidth(), vector<myValType>( img->getHeight() ));
        //Just give pixel values to the map and be done with it.
        for( size_t x = 0; x < img->getWidth(); ++x)
        {
            for( size_t y = 0; y < img->getHeight(); ++y)
                ( myMap.at(x) ).at(y) = (myValType)( img->getPixelValuei(x,y,0) );
            ( myMap.at(x) ).shrink_to_fit();
        }
    }
};

using brushfire_map = brushfireMap;
using wavefront_map = wavefrontMap;
using pixelshade_map = pixelshadeMap;



class testMap : public brushfireMap {
public:
    testMap(shared_ptr<Image> img, const pos_t &reachableFreeSpace)
        :brushfireMap(img,reachableFreeSpace)
    {}

    void test(shared_ptr<Image> img)
    {
        set<pos_t> loma;
        for(coordIndexType x=0;x<getWidth();++x)
        {
            coordIndexType y0=2, y1=1, y2=0;

            while(y0<getHeight())
            {
                if(
                        ((this->const_coordVal(x,y0))<(this->const_coordVal(x,y1)))
                        &&((this->const_coordVal(x,y2))<(this->const_coordVal(x,y1)))
                        )
                    loma.emplace(x,y1);
                ++y0;
                ++y1;
                ++y2;
            }
        }
        for(coordIndexType y=0;y<getHeight();++y)
        {
            coordIndexType x0=2, x1=1, x2=0;

            while(x0<getWidth())
            {
                if(
                        ((this->const_coordVal(x0,y))<(this->const_coordVal(x1,y)))
                        &&((this->const_coordVal(x2,y))<(this->const_coordVal(x1,y)))
                        )
                    loma.emplace(x1,y);
                ++x0;
                ++x1;
                ++x2;
            }
        }
        for(auto i:loma)
            img->setPixel8U(i.cx(),i.cy(),0);
    }
};

/**
 * @brief The dijkstraMap class Same as integer wavefront, except it's norm2 potential function.
 */
class dijkstraMap : public tekMap<double> {
public:
    dijkstraMap(shared_ptr< Image > img, const list<pos_t> &coords = list< pos_t >())
        :tekMap(img)
    {
        if(coords.empty())
            cerr << "No coords passed to dijkstraMap constructor!" << endl;
        wave(img,coords);
    }

    using edgeType = pair<pos_t,myValType>;
    struct edgeComp {
        bool operator() ( const edgeType &lhs, const edgeType &rhs) const
        { return (lhs.second)>(rhs.second); }
    };
    /**
     * @brief wave Stores all Dijkstra's algorithm values in myMap.
     * @param img Image pointer
     * @param goals set of goal coordinates
     */
    virtual void wave(shared_ptr<Image> img, const list<pos_t> &goals) override
    {
        const double sqrt2 =1.4142135623730950488016887242097;
        const array<array<int,2>,8> neighbours =
        {{  {-1,0}, /* W */ {1,0},  /* E */
            {0,-1}, /* N */ {0,1},  /* S */
            {-1,-1},/* NW */{1,-1}, /* NE */
            {1,1},  /* SE */{-1,1}  /* SW */
         }};
        myMap.clear();
        //4000 looks nicer when printed, but any high value (INF) works:
        myMap.resize( img->getWidth(), vector<myValType>( img->getHeight() , 4000));
        //myMap.resize( img->getWidth(), vector<myValType>( img->getHeight() , numeric_limits<myValType>::max()));
        vector<vector<bool> > visited(img->getWidth(),vector<bool>(img->getHeight(),false));
        priority_queue<edgeType,deque<edgeType>,edgeComp > q;
        for(auto i : goals) {
            (myMap[i.cx()])[i.cy()] = WAVE_VAL_GOAL;
            q.emplace(i,(myMap[i.cx()])[i.cy()]);
        }
        while(!q.empty())
        {
            edgeType cur = q.top();
            q.pop();
            visited[cur.first.cx()][cur.first.cy()]=true;
            for(auto n : neighbours) {
                //If neighbour is within image borders:
                if( isInImage( img, cur.first.cx()+n.at(0), cur.first.cy()+n.at(1) ) ) {
                    //if the neighbour is an unvisited non-obstacle:
                    if( ( !WSPACE_IS_OBSTACLE( img->getPixelValuei(cur.first.cx()+n.at(0),cur.first.cy()+n.at(1),0) ) )
                            && ( !( (visited[cur.first.cx()+n.at(0)])[cur.first.cy()+n.at(1)] ) )
                            && ((myMap[cur.first.cx()+n.at(0)])[cur.first.cy()+n.at(1)] != WAVE_VAL_GOAL)
                            )
                    {
                        if(
                                ((myMap[cur.first.cx()])[cur.first.cy()]
                                 +(((n.at(0)==0) || (n.at(1)==0))?1.0:sqrt2))
                                <((myMap[cur.first.cx()+n.at(0)])[cur.first.cy()+n.at(1)])
                                ) {
                            //increment:
                            (myMap[cur.first.cx()+n.at(0)])[cur.first.cy()+n.at(1)]
                                    = (myMap[cur.first.cx()])[cur.first.cy()]
                                    +(((n.at(0)==0) || (n.at(1)==0))?1.0:sqrt2);
                            //Add it to the wavefront
                            q.emplace(pos_t(cur.first.cx()+n.at(0),cur.first.cy()+n.at(1)),
                                      (myMap[cur.first.cx()+n.at(0)])[cur.first.cy()+n.at(1)]);
                        }
                    }
                }
            }
        }
    }
};
