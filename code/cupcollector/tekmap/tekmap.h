/** @file
 * @author Lukas, Michael, Mikkel, Nikolaj & Mikael
 * @date 21-10-2014
 *
 * Group project.
*/
#pragma once

#include "libraries/Image.hpp"
#include <memory>
#include <vector>
#include <array>
#include <utility>
#include <set>

using namespace rw::sensor;
using namespace std;

class tekMap
{
public:
    using coordIndexType = unsigned int;
    /** @brief The value type for each pixel */
    using coordValType = long int;

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
    /** @brief posType aliases the integer coordinate type (a pair of uints) */

    /** @brief mapType denotes the different map types... */
    typedef enum {UNINITIALIZED, WAVEFRONT, BRUSHFIRE, PIXELSHADE} mapType;

    /**
     * @brief map Constructor
     * @param img Pointer to the image
     * @param myType The type of map you want to construct
     * @param coords Pointer to set of offloading station coordinates.
     * @param reachableFreeSpace a coordinate inside the MAP, which the robot can reach.
     *
     * If reachableFreeSpace is supplied, the border-finding for the Brushfire
     * algorithm will be faster. Thus, it's only useful to supply it for Brushfire maps.
     */
    tekMap(shared_ptr< Image > img, const mapType argMyType = UNINITIALIZED,
        set<pos_t> coords = set< pos_t >() , const pos_t *reachableFreeSpace = nullptr);

    /** @brief getType returns the type of map...*/
    inline mapType getType() const;

    /** @brief getCoordVal returns the coordinate value... */
    inline coordValType &const_coordVal(const pos_t::x_type &x, const pos_t::y_type &y) const;
    inline coordValType &const_coordVal(pos_t::x_type &&x, pos_t::y_type &&y) const;
    inline coordValType &coordVal(const pos_t::x_type &x, const pos_t::y_type &y);
    inline coordValType &coordVal(pos_t::x_type &&x, pos_t::y_type &&y);
    inline coordValType &const_coordVal(const pos_t &ofThisCoord) const;
    inline coordValType &const_coordVal(pos_t &&ofThisCoord) const;
    inline coordValType &coordVal(const pos_t &ofThisCoord);
    inline coordValType &coordVal(pos_t &&ofThisCoord);




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
    inline bool isInImage(const shared_ptr<Image> img, const int &x, const int &y) const;

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
    virtual set< pos_t > findObstacleBorders(shared_ptr<Image> img) const;

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
    virtual inline set< pos_t > findObstacleBorders(shared_ptr<Image> img, const pos_t &validFreeSpaceCoord) const;

    /**
     * @brief wave I have no idea if this works. But if it does, it's awesome.
     * @param img Image pointer
     * @param goals set of goal coordinates
     */
    virtual void wave(shared_ptr<Image> img, const set<pos_t> &goals);

    /**
     * @brief findCoords Finds workspace (freespace) coordinates or relevant borders.
     * @param img Image pointer
     * @param withinFreeSpace A coordinate within the freespace, eg. a valid coordinate.
     * @param borders If true, only coords bordering the freespace are returned,
     *                if false, only coords within the freespace are returned.
     * @return Coords bordering or within the freespace, depending on the borders parameter.
     */
    virtual set< pos_t > findCoords(shared_ptr<Image> img, const pos_t &withinFreeSpace, const bool borders=true) const;

};
