/** @file
* @author Lukas, Michael, Mikkel, Nikolaj & Mikael
* @date 21-10-2014
*
* Group project.
*/
#pragma once
#include "tekmap.hpp"
#include "pixelshade.h"
#include "../libraries/Image.hpp"
#include <list>

using namespace std;
using namespace rw::sensor;

class brushfireMap;
class norm2BrushfireMap;
using brushfire_map = brushfireMap;
using norm2_brushfire_map = norm2BrushfireMap;

/**
 * @brief The brushfireMap class Stores integer brushfire algorithm values.
 * The algorithm's potential function is normInf.
 */
class brushfireMap : public tekMap<unsigned char>
{
protected:
	/**
	 * @brief findCoords Finds either freespace coordinates or border coordinates
	 * Propagates a wave. If you want borders, the coordinates are stored
	 * when the wavefront hits the borders.
	 * If you want freespace, the coordinates are stored as the wave propagates
	 * through them.
	 *
	 * @param img Image
	 * @param withinFreeSpace A coordinate within the freespace
	 * @param borders True if you want borders. False if you want freespace.
	 * @return List of coordinates.
	 */
	virtual list< pos_t > findCoords(const pixelshadeMap &img, const pos_t &withinFreeSpace, const bool borders=true) const;

	/**
	 * @brief findObstacleBorders returns all obstacle borders in image.
	 * @param img The image to find obstacle borders in.
	 * @return set of obstacle border coordinates.
	 * How it works:
	 * For all pixels in image:
	 * if pixel is an obstacle:
	 * if pixel has at least one freespace-neighbour:
	 * return_value.insert(posType(x,y));
	 * return return_value.
	 *
	 * So this is O(N).
	 */
	virtual list< pos_t > findObstacleBorders(shared_ptr<Image> img) const;
	virtual list< pos_t > findObstacleBorders(const pixelshadeMap &img) const;

	/**
	 * @brief findObstacleBorders returns all RELEVANT obstacle borders in image.
	 * @param img The image to find relevant obstacle borders in.
	 * @param validFreeSpaceCoord A free space coordinate the robot can reach.
	 * @return set of relevant obstacle border coordinates.
	 * How it works:
	 * Do
	 * Propagate wave from validFreeSpaceCoord.
	 * If wave hits an obstacle pixel:
	 * return_value.insert(obstacle_pixel);
	 * until wave has visited all freespace.
	 * return return_value.
	 *
	 * So this is O(N) but performs better than the other findObstacleBorders.
	 */
	virtual inline list< pos_t > findObstacleBorders(shared_ptr<Image> img, const pos_t &validFreeSpaceCoord) const
	{ return move(findCoords(img,validFreeSpaceCoord,true)); }
	virtual inline list< pos_t > findObstacleBorders(const pixelshadeMap &img, const pos_t &validFreeSpaceCoord) const
	{ return move(findCoords(img,validFreeSpaceCoord,true)); }
public:
	/**
	 * @brief brushfireMap Constructor
	 * @param img Image
	 */
	brushfireMap(shared_ptr< Image > img);
	brushfireMap(const pixelshadeMap &img);
	/**
	 * @brief brushfireMap Constructor. Will only spend time on values in freespace.
	 * @param img Image
	 * @param reachableFreeSpace One coordinate in reachable freespace.
	 */
	brushfireMap(shared_ptr< Image > img, const pos_t &reachableFreeSpace);
	brushfireMap(const pixelshadeMap &img, const pos_t &reachableFreeSpace);

	/**
	 * @brief wave Wavefront algorithm for pixelshadeMap. Like wave() of tekMap.
	 * Refer to tekMap::wave(), as the only difference is the image type.
	 * @param img Image
	 * @param goals Goal coordinates (borders in brushfire maps)
	 */
	virtual void wave(const pixelshadeMap &img, const list<pos_t> &goals);

};



/**
 * @brief The norm2BrushfireMap class Brushfire map using norm2 potential function.
 */
class norm2BrushfireMap : public tekMap<double> {
protected:
	/**
	 * @brief findObstacleBorders Finds all obstacle borders
	 * @param img Image to find obstacle borders in
	 * @return List of borders. No duplicates (MW-guaranteed)
	 */
	virtual list< pos_t > findObstacleBorders(const pixelshadeMap &img) const;

	/**
	 * @brief wave Norm2 Dijkstra's algorithm propagation for pixelshadeMaps.
	 * @param img Image to propagate wave in
	 * @param goals The goals to propagate from (the obstacle borders!)
	 */
	virtual void wave(const pixelshadeMap &img, const list<pos_t> &goals);

public:
	/**
	 * @brief norm2BrushfireMap Constructor. Just pass a pixelshadeMap.
	 * @param img A pixelshadeMap.
	 */
	norm2BrushfireMap(const pixelshadeMap& img);

};
