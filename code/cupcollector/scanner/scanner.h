/** @file
 * @author Lukas, Michael, Mikkel, Nikolaj & Mikael
 * @date 21-10-2014
 *
 * Group project.
*/

#pragma once

#include <iostream>
#include "../tekmap/tekmap.hpp"
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
				unsigned int radius=ROBOT_SCANNER_RADIUS);
private:
	unsigned int r = -1;
	forward_list<pos_t> circle;
};
