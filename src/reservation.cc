#include "seat/reservation.h"

#include <cassert>
#include <algorithm>
#include <iostream>

#include "utl/enumerate.h"
#include "utl/zip.h"

namespace seat {

std::ostream& operator<<(std::ostream& out, reservation const& r) {
  for (auto const& w : r) {
    out << w;
  }
  return out;
}

bool matches(reservation const a, reservation const b) {
  for (auto const& [pa, pb] : utl::zip(a, b)) {
    if (pa != wish::kAny && pa != pb) {
      return false;
    }
  }
  return true;
}

int concreteness(reservation r){
  return std::count_if(begin(r),end(r),[](wish w){
    return w==wish::kAny;
  });
}

uint8_t to_int(reservation r){
  auto sum = 0U;
  for(auto const [w_id,w]:utl::enumerate(r)){
    if(w==wish::kYes) {
      sum |= 1 << w_id;
    }
  }
  return sum;
}

bool matches(reservation r, uint8_t other){
  for(auto const [w_idx,w]:utl::enumerate(r)){
    if (w == wish::kAny ){
      continue;
    }
    if(((other>>w_idx)&1)!=(w==wish::kYes?1:0)){
      return false;
    }
  }
}

}  // namespace seat
