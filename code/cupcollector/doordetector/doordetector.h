/** @file */
#pragma once
#include "../tekmap/tekmap.hpp"
#include <vector>


class doorDetector {
protected:
    std::vector<pos_t> reduce_number_of_doors (const std::vector<pos_t> &large_door_list , const brushfire_map &brushmap);
    bool doorway_check ( pos_t pos, const brushfire_map &brushmap );
public:
    std::vector<pos_t> detect_doorways(const brushfire_map &brushmap );
};
