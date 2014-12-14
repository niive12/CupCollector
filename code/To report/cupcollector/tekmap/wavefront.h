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
#include <vector>
#include <set>


using namespace std;
using namespace rw::sensor;

class wavefrontMap;
class dijkstraMap;
using wavefront_map = wavefrontMap;
using dijkstra_map = dijkstraMap;

/**
 * @brief The wavefrontMap class Stores integer wavefront algorithm values.
 * The algorithm uses normInf potential function (integers).
 */
class wavefrontMap : public tekMap<long unsigned int>
{
public:
	/**
	 * @brief wavefrontMap Constructor
	 * @param img Image to construct from.
	 * @param coords Coordinates to propagate wave from (the offloading stations)
	 */
	wavefrontMap(shared_ptr< Image > img, const list<pos_t> &coords = list< pos_t >());
};

/**
 * @brief The dijkstraMap class Same as integer wavefront, except it's norm2 potential function.
 */
class dijkstraMap : public tekMap<double> {
public:
	/**
	 * @brief dijkstraMap Constructor.
	 * @param img Image to construct from
	 * @param coords Coordinates to propagate wave from (the offloading stations)
	 */
	dijkstraMap(shared_ptr< Image > img, const list<pos_t> &coords = list< pos_t >());

	/**
	 * @brief wave Stores all Dijkstra's algorithm values in myMap.
	 * @param img Image pointer
	 * @param goals set of goal coordinates
	 */
	virtual void wave(shared_ptr<Image> img, const list<pos_t> &goals) override;

	/**
	 * @brief getShortestPath Using the internal map, finds the shortest path to a goal (offloading station)
	 * @param from The starting coordinate.
	 * @return Coordinates between from and the goal, including goal (not including from).
	 */
	virtual list<pos_t> getShortestPath(const pos_t &from);
};
