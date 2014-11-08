/** @file */
#pragma once
#include "tekmap/tekmap.hpp"
#include <vector>


class doorDetector {
protected:
    bool doorway_check (shared_ptr<Image> img, pos_t pos, const brushfire_map &brushmap);
public:
    std::vector<pos_t> detect_doorways(shared_ptr<Image> img, const brushfire_map &brushmap );
    pixelshade_map door_step(shared_ptr<Image> img, brushfire_map &brushmap , std::vector<pos_t> the_doors);
};
