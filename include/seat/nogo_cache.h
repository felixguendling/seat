#pragma once

#include <map>
#include <set>

#include "seat/interval.h"
#include "seat/reservation.h"

namespace seat {

struct booking;

struct nogo_cache {
  void add_entry(booking const&);
  bool is_nogo(booking const&) const;

private:
  std::map<reservation, std::set<interval>> nogo_intervals_;
};

}  // namespace seat
