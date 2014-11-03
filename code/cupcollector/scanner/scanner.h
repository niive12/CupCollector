/** @file
 * @author Lukas, Michael, Mikkel, Nikolaj & Mikael
 * @date 21-10-2014
 *
 * Group project.
*/

#pragma once

#include <iostream>
#include "tekmap/tekmap.hpp"
#include <forward_list>

using namespace std;


class scanner
{
public:
    /**
     * @brief scanner Constructor
     * @param x0 x-coordinate of center of circle
     * @param y0 y-coordinate of center of circle
     * @param r the radius of the circle in pixels
     *
     * Will allocate an array with all the coordinate inside the circle with center (x0,y0) and radius r.
     * Will make coordinate_array point tot the array.
     */
    scanner(unsigned int x0, unsigned int y0, int r);

    /**
     * @brief Scanner function
     * @param x0 x-coordinate of center of circle
     * @param y0 y-coordinate of center of circle
     *
     * Not finished. Should not be void!
     */
    void scan(unsigned int x0, unsigned int y0);

    /**
     * @brief scanner Destructor
     *
     * Deletes the dynamic allocated array
     */
    ~scanner();

private:
    /** @brief coordinate_array will point to an array with all coordinate inside the circle, when the constructor has run */
    pos_t* coordinate_array = nullptr;
    //auto coordinate_array = make_shared<tekMap::pos_t>;
};



struct cupScanner {
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
