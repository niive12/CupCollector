/*
 * macros.hpp
 *
 *  Created on: Aug 30, 2012
 *      Author: jimali
 *
 *   Modified on: Sep 20, 2014
 *        Author: miwes12
 *  Modification: __LINE__ is now output.
 */

#ifndef MACROS_HPP_
#define MACROS_HPP_

#include <sstream>
#include <iostream>


#define RW_THROW(ostreamExpression) do { int RW__line = __LINE__;           \
    std::stringstream RW__stream;                                           \
    RW__stream << RW__line << ostreamExpression;                                        \
    std::cerr << RW__line << ostreamExpression; \
    throw RW__stream.str();                               \
} while (0)

#define RW_WARN(ostreamExpression) do { int RW__line = __LINE__;           \
    std::stringstream RW__stream;                                           \
    RW__stream << RW__line << ostreamExpression;                                        \
    std::cout << RW__line << ostreamExpression; \
    throw RW__stream.str();                               \
} while (0)


#endif /* MACROS_HPP_ */
