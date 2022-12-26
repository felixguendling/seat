#include "seat/nogo_cache.h"

#include "seat/booking.h"

namespace seat {

void remove_containing_intervals(interval const& i, std::set<interval>& s) {
  auto it = begin(s);
  while (it != end(s)) {
    if (it->contains(i)) {
      it = s.erase(it);
    } else {
      ++it;
    }
  }
}

bool interval::contains(interval const& o) const {
  return o.from_ >= from_ && o.to_ <= to_;
}

bool interval::overlaps(interval const& o) const {
  // 1-2 vs 2-3: 1 < 3 && 2 > 2 => false
  // 2-3 vs 1-2: 2 < 2 && 3 > 1 => false
  // 1-3 vs 2-4: 1 < 4 && 3 > 2 => true
  return from_ < o.to_ && to_ > o.from_;
}

void nogo_cache::add_entry(booking const& b) {
  auto& r_nogo_intervals = nogo_intervals_[b.r_];
  remove_containing_intervals(b.interval_, r_nogo_intervals);
  r_nogo_intervals.emplace(b.interval_);
}

bool nogo_cache::is_nogo(booking const& b) const {
  auto it = nogo_intervals_.find(b.r_);
  if (it == end(nogo_intervals_)) {
    return false;
  }

  auto& r_nogo_intervals = it->second;
  for (auto const& nogo : r_nogo_intervals) {
    if (b.interval_.contains(nogo)) {
      return true;
    }
  }

  return false;
}

}  // namespace seat
