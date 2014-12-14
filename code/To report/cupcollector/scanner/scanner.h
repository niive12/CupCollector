/** @file
 * @author Lukas, Michael, Mikkel, Nikolaj & Mikael
 * @date 21-10-2014
 *
 * Group project.
*/

#pragma once

#include <iostream>
#include "../tekmap/pixelshade.h"
#include <list>
#include <unordered_set>
#include <forward_list>

using namespace std;

struct scanner {
	/**
	 * @brief scan Returns the number of cups within scanner radius.
	 * @param center Center of scanner circle.
	 * @param scanmap The pixel map to scan in
	 * @param radius Radius in pixels of scanner.
	 * @return Number of cups within radius.
	 */
	size_t scan(pos_t center, const pixelshade_map &scanmap,
				unsigned int radius);
	size_t scan(pos_t center, const pixelshade_map &scanmap);

	unsigned int radius();

	/**
	 * @brief scanlistThroughWalls Returns positions of all cups within the circle
	 * @param center Center of circle to scan
	 * @param scanmap Map to scan for cups in
	 * @param radius Radius of circle
	 * @return List of positions of all cups within circle with radius radius.
	 */
	list<pos_t> scanlistThroughWalls(pos_t center, const pixelshade_map &scanmap,
						 unsigned int radius);
	list<pos_t> scanlistThroughWalls(pos_t center, const pixelshade_map &scanmap);

	/**
	 * @brief scanlistAroundWalls Returns positions of all cups that can be reached from center of circle.
	 * @param center Center of circle to scan
	 * @param scanmap Map to scan for cups in
	 * @param radius Radius of circle
	 * @return List of positions of all cups to which there are paths from center.
	 */
	list<pos_t> scanlistAroundWalls(pos_t center, const pixelshade_map &scanmap,
						 unsigned int radius);
	list<pos_t> scanlistAroundWalls(pos_t center, const pixelshade_map &scanmap);

	/**
	 * @brief scanlistLineOfSight Returns positions of all cups that are within line of sight of center of circle
	 * @param center Center of circle to scan
	 * @param scanmap Map to scan for cups in
	 * @param radius Radius of circle
	 * @return List of positions of all cups within circle, that are within LOS of center coordinate.
	 */
	list<pos_t> scanlistLineOfSight(pos_t center, const pixelshade_map &scanmap,
						 unsigned int radius);
	list<pos_t> scanlistLineOfSight(pos_t center, const pixelshade_map &scanmap);

	/**
	 * @brief scanner Constructor with initialization of data members
	 * @param radius Radius of scanner circle
	 */
	scanner(unsigned int radius);

	/**
	 * @brief scanner Constructor with no initialization.
	 */
	scanner();

	/**
	 * @brief getCircle Absolute coordinates of scanner circle
	 * @param center Center of circle
	 * @param radius Radius of circle
	 * @return All coordinates within circle.
	 */
	unordered_set<pos_t> getCircle(const pos_t &center, unsigned int radius);

private:
	unsigned int r = -1;
	forward_list<pos_t> circle;

	/**
	 * @brief update_circle Updates the circle's relative coordinates
	 * @param radius Radius of new circle
	 */
	void update_circle(unsigned int radius);

	/**
	 * @brief getCircle Absolute coordinates of scanner circle
	 * @param center Center of circle
	 * @return All coordinates within circle.
	 */
	unordered_set<pos_t> getCircle(const pos_t &center);

	/**
	 * @brief isVisible Attempts to find an unblocked line between two coordinates
	 * @note Discretizes coordinates on straight line by rounding to nearest integers x and y.
	 *
	 * @param scanmap Image to scan in
	 * @param cup The "to" coordinate
	 * @param from The "from" coordinate (center)
	 * @return True if no obstacles were found on the straight line between cup and from.
	 */
	void getCupsInSight(const pixelshadeMap &scanmap, set<pos_t> &cups, const pos_t &to, const pos_t &from);

	/**
	 * @brief getPerimeter Returns all coordinates on circle perimeter
	 * @param center Center of circle
	 * @param radius Radius of circle
	 * @return List of coordinates on circle perimeter (r-1)*(r-1) < ((dx*dx)+(dy*dy)) < r*r
	 */
	forward_list<pos_t> getPerimeter(const pos_t &center, unsigned int radius);

};
