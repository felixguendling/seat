#include "seat/interval_graph.h"

#include <algorithm>

#include "utl/enumerate.h"

namespace seat {

interval_graph::interval_graph(std::vector<booking> const& bookings)
    : all_bookings_(bookings) {}

void interval_graph::create_graph(std::vector<booking_id_t> const& b_ids) {
  b_ids_ = b_ids;
  if (b_ids_.size() == 0) {
    return;
  }
  neighbours_.resize(b_ids_.size());
  for (auto const& [idx, b_id] : utl::enumerate(b_ids_)) {
    auto interv1 = all_bookings_[b_id].interval_;
    neighbours_[idx] = std::vector<int>();
    for (auto const& [idx2, b_id2] : utl::enumerate(b_ids_)) {
      if (idx == idx2) {
        continue;
      }
      auto interv2 = all_bookings_[b_id2].interval_;
      // check overlap
      if ((interv1.from_ >= interv2.from_ && interv2.to_ > interv1.from_) ||
          (interv2.from_ >= interv1.from_ && interv1.to_ > interv2.from_) ||
          (idx >= b_ids.size() && idx2 >= b_ids.size())) {
        neighbours_[idx].emplace_back(idx2);
      }
    }
  }
  seats_by_booking_id_.resize(b_ids_.size());
  fill(seats_by_booking_id_.begin(), seats_by_booking_id_.end(),
       std::numeric_limits<seat_id_t>::max());
  auto used_seats =
      neighbours_[std::max_element(
                      begin(neighbours_), end(neighbours_),
                      [&](std::vector<int> v1, std::vector<int> v2) {
                        return v1.size() < v2.size();
                      }) -
                  begin(neighbours_)]
          .size() +
      1;
  auto blocked = std::vector<int>();
  blocked.resize(used_seats);
  fill(begin(blocked), end(blocked), -1);
  auto todos = std::vector<booking_id_t>();
  auto done = std::vector<booking_id_t>();
  while (seats_by_booking_id_[std::max_element(seats_by_booking_id_.begin(),
                                               seats_by_booking_id_.end()) -
                              begin(seats_by_booking_id_)] ==
         std::numeric_limits<seat_id_t>::max()) {
    int min = std::numeric_limits<int>::max();
    int min_idx = -1;
    for (auto const& [idx, b_id] : utl::enumerate(b_ids_)) {
      if (seats_by_booking_id_[idx] != std::numeric_limits<seat_id_t>::max()) {
        continue;
      }
      if (min <= all_bookings_[b_id].interval_.from_) {
        continue;
      }
      min = all_bookings_[b_id].interval_.from_;
      min_idx = idx;
    }
    todos.emplace_back(min_idx);
    while (!todos.empty()) {
      auto current_todo = todos[0];
      for (auto const& [idx, block] : utl::enumerate(blocked)) {
        if (block > all_bookings_[b_ids_[current_todo]].interval_.from_) {
          continue;
        }
        blocked[idx] = all_bookings_[b_ids_[current_todo]].interval_.to_;
        seats_by_booking_id_[current_todo] = idx;
        for (auto const& neighbour : neighbours_[current_todo]) {
          if (find(begin(todos), end(todos), neighbour) != end(todos) ||
              find(done.begin(), done.end(), neighbour) != done.end()) {
            continue;
          }
          todos.emplace_back(neighbour);
        }
        done.emplace_back(current_todo);
        todos.erase(todos.begin());

        std::sort(todos.begin(), todos.end(),
                  [&](booking_id_t b1, booking_id_t b2) {
                    return all_bookings_[b_ids_[b1]].interval_.from_ <
                           all_bookings_[b_ids_[b2]].interval_.from_;
                  });
        break;
      }
    }
  }
}

}  // namespace seat
