/** @file
 * @author Lukas, Michael, Mikkel, Nikolaj & Mikael
 * @date 21-10-2014
 *
 * Group project.
*/

#pragma once

#include <iostream>
#include "tekmap/tekmap.h"

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
    tekMap::posType* coordinate_array = nullptr;
    //auto coordinate_array = make_shared<tekMap::posType>;
};
