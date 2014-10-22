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
    /** @brief posType aliases the integer coordinate type (a pair of uints) */
    using posType = pair< unsigned int, unsigned int >;

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
    tekMap(shared_ptr< Image > img, mapType argMyType = UNINITIALIZED,
        set<posType> coords = set< posType >() , const posType *reachableFreeSpace = nullptr);

    /** @brief getType returns the type of map...*/
    inline mapType getType();

protected:
    /** @brief myType is this map's type*/
    mapType myType;

    /** @brief The value type for each pixel */
    using coordValType = long int;

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
    set< posType > findObstacleBorders(shared_ptr<Image> img);

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
    set< posType > findObstacleBorders(shared_ptr<Image> img, const posType validFreeSpaceCoord);

    /**
     * @brief wave I have no idea if this works. But if it does, it's awesome.
     * @param img Image pointer
     * @param goals set of goal coordinates
     */
    void wave(shared_ptr<Image> img, const set<posType> &goals);

};
