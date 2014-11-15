/** @file
* @author Lukas, Michael, Mikkel, Nikolaj & Mikael
* @date 21-10-2014
*
* Group project.
*/
#pragma once
#include "tekmap.hpp"
#include "../libraries/Image.hpp"
#include <list>
#include <unordered_set>


using namespace std;
using namespace rw::sensor;

class pixelshadeMap;
using pixelshade_map = pixelshadeMap;

/**
 * @brief The pixelshadeMap class tekMap representation of Image.
 */
class pixelshadeMap : public tekMap<unsigned char>
{
public:
	/**
	 * @brief pixelshadeMap Constructor
	 * @param img Image to copy into this pixelshadeMap.
	 */
	pixelshadeMap(shared_ptr< Image > img);

	/**
	 * @brief getWavefrontPath Finds the wavefront algorithm shortest path.
	 * @param from Starting coordinate
	 * @param to Goal coordinate
	 * @return List of coordinates between from and to.
	 */
	list<pos_t> getWavefrontPath(const pos_t &from, const pos_t &to) const;

	/**
	 * @brief getDijkstraPath Finds the Dijkstra's weighted shortest path.
	 * @param from Starting coordinate
	 * @param to Goal coordinate
	 * @return List of coordinates between from and to.
	 */
	list<pos_t> getDijkstraPath(const pos_t &from, const pos_t &to) const;

	/**
	 * @brief findFreespace Finds freespace connected to a certain coordinate.
	 * @param withinFreeSpace One coordinate within the freespace you want to find.
	 * @return All coordinates, including withinFreespace, within freespace.
	 */
	virtual unordered_set< pos_t > findFreespace(const pos_t &withinFreeSpace) const;
};
