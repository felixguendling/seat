#include "seat/reservation.h"

#include <cassert>
#include <algorithm>

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

int concreteness(reservation r) {
  return std::count_if(begin(r), end(r),
                       [](wish w) { return w == wish::kAny; });
}

encoded_reservation_t to_int(reservation r) {
  auto sum = 0U;
  for (auto const [w_id, w] : utl::enumerate(r)) {
    if (w == wish::kYes) {
      sum |= 1 << w_id;
    }
  }
  return sum;
}

encoded_reservation_t to_int_all(reservation r) {
  auto sum = 0U;
  for (auto const [w_id, w] : utl::enumerate(r)) {
    if (w == wish::kYes) {
      sum |= 1 << w_id;
    }
    if (w == wish::kAny) {
      sum |= 1 << (w_id + r.size());
    }
  }
  return sum;
}

bool matches(reservation r, encoded_reservation_t other) {
  for (auto const [w_idx, w] : utl::enumerate(r)) {
    if (w == wish::kAny) {
      continue;
    }
    if (((other >> w_idx) & 1) != (w == wish::kYes ? 1 : 0)) {
      return false;
    }
  }
  return true;
}
/*
template <typename F>
void for_each_concrete_reservation(reservation const& r, F&& f) {
  std::vector<int> any_ids;
  reservation copy = r;
  for (int i = 0; i != r.size(); ++i) {
    if (r[i] == wish::kAny) {
      any_ids.emplace_back(i);
    }
    copy[i] = wish::kYes;
  }
  for (uint16_t i = 0; i != exp2(any_ids.size()); ++i) {
    for (int j = 0; j != r.size(); ++j) {
      copy[j] = ((static_cast<uint16_t>(i / exp2(j)) % 2U) == 0) ? wish::kYes
                                                                 : wish::kNo;
    }
    f(copy);
  }
}
*/
std::vector<reservation> get_concrete_reservations(reservation const& r) {
  std::vector<reservation> concrete_reservations;
  std::vector<int> any_ids;
  reservation copy = r;
  for (int i = 0; i != r.size(); ++i) {
    if (r[i] == wish::kAny) {
      any_ids.emplace_back(i);
      copy[i] = wish::kYes;
    }
  }
  for (uint16_t i = 0; i != exp2(any_ids.size()); ++i) {
    for (int j = 0; j != any_ids.size(); ++j) {
      copy[any_ids[j]] = ((static_cast<uint16_t>(i / exp2(j)) % 2U) == 0)
                             ? wish::kYes
                             : wish::kNo;
    }
    auto copy2 = copy;
    concrete_reservations.emplace_back(copy2);
  }
  return concrete_reservations;
}

std::map<reservation, std::uint32_t> generate_number_of_seats(
    std::vector<std::uint32_t> const& seats, std::uint8_t const wish_types) {
  auto seats_by_reservation = std::map<reservation, std::uint32_t>{};
  auto all_any_res = reservation{};
  for (auto i = 0; i != std::min(uint8_t{4}, wish_types); ++i) {
    all_any_res[i] = wish::kAny;
  }
  auto concrete_res = get_concrete_reservations(all_any_res);
  assert(concrete_res.size() == seats.size());
  for (auto i = 0; i != concrete_res.size(); ++i) {
    seats_by_reservation.emplace(concrete_res[i], seats[i]);
  }
  return seats_by_reservation;
}

std::vector<reservation> get_concrete_reservations(uint8_t const n) {
  reservation anys;
  std::fill(anys.begin(), anys.end(), wish::kNo);
  for (int i = 0; i != n; ++i) {
    anys[i] = wish::kAny;
  }
  return get_concrete_reservations(anys);
}
}  // namespace seat
