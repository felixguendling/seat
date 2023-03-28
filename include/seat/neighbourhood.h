#pragma once
#include <vector>

#include "utl/enumerate.h"

#include "seat/booking.h"
#include "seat/types.h"

namespace seat {
struct neighbourhood{

    bool is_reverse(const neighbourhood nh) const {
      return moved_==nh.moved_&&old_color_==nh.new_color_;
    }
   node_id_t moved_;
   color_t old_color_,new_color_;
};
}