#include "seat/heuristic_state.h"
#include <algorithm>

#include "seat/booking.h"
#include "seat/interval.h"

namespace seat {

void heuristic_state::insert_booking(booking new_booking) {
  if (!colors_.empty() &&
      std::max_element(colors_.begin(), colors_.end())[0] > 1000) {
    auto s = "";
  }
  bookings_.emplace_back(new_booking);
  update_overlaps(new_booking);
  auto const new_booking_id = bookings_.size() - 1;

  auto const valid_spots = check_gsd_overlaps(new_booking_id);
  possible_pos_.emplace_back(valid_spots);
  tabu_count_.emplace_back(0U);
  feasible_ = valid_spots > 0 ? UNKNOWN : INFEASIBLE;

  if (feasible_ == INFEASIBLE) {
    // problem is invalid, as the new new_booking cannot find any slot even if
    // only considering gsd bookings.
    return;
  }
  --num_valid_neighbours;
  move_to_best_location(new_booking_id);
  feasible_ = (illegal_count_ == 0) ? FEASIBLE : UNKNOWN;
}

bool heuristic_state::move_to_best_location(node_id_t const new_booking_id) {
  auto overlaps_by_color = std::vector<uint16_t>();
  overlaps_by_color.resize(wish_combination_by_color_.size());
  std::fill(overlaps_by_color.begin(), overlaps_by_color.end(),
            overlaps_[new_booking_id].size() + 2);
  for (auto const overlapping_booking_id : overlaps_[new_booking_id]) {
    overlaps_by_color[colors_[overlapping_booking_id]]--;
  }
  auto max = 0;
  auto const cmp = colors_.size() * wish_combination_by_color_.size() * 100 + 1;
  auto best_color = cmp;
  for (auto color = color_t{0}; color != wish_combination_by_color_.size();
       ++color) {
    if (!matches(bookings_[new_booking_id].r_,
                 wish_combination_by_color_[color]) ||
        is_gsd_blocked(color, bookings_[new_booking_id].interval_)) {
      continue;
    }
    if (overlaps_by_color[color] > max) {
      max = overlaps_by_color[color];
      best_color = color;
    }
  }
  illegal_count_ += count_new_overlaps(new_booking_id, best_color);
  if (best_color == cmp) {
    return false;
  }
  if (new_booking_id < colors_.size()) {
    colors_[new_booking_id] = best_color;
  } else {
    colors_.emplace_back(best_color);
  }
  return true;
}

void heuristic_state::insert_gsd_booking(color_t new_color,
                                         interval new_interval) {
  feasible_ = is_gsd_blocked(new_color, new_interval) ? INFEASIBLE : UNKNOWN;
  if (feasible_ == INFEASIBLE) {
    return;
  }
  gsd_intervals_by_color_[new_color].emplace_back(new_interval);
  for (auto node_id = color_t{0}; node_id != colors_.size(); ++node_id) {
    if (colors_[node_id] != new_color) {
      continue;
    }
    if (new_interval.overlaps(bookings_[node_id].interval_)) {
      possible_pos_[node_id]--;
      if (possible_pos_[node_id] == 0) {
        feasible_ = INFEASIBLE;
        return;
      }
      if (!move_to_best_location(node_id)) {
        feasible_ = INFEASIBLE;
        return;
      }
      illegal_count_ -= count_new_overlaps(node_id, colors_[node_id]);
    }
  }
  if (uint16_t{0} == illegal_count_) {
    feasible_ = FEASIBLE;
  }
}

void heuristic_state::update_overlaps(booking new_booking) {
  auto const new_booking_id = bookings_.size() - 1;
  std::vector<node_id_t> new_booking_overlaps;
  for (auto const [id, booking] : utl::enumerate(bookings_)) {
    if (id == new_booking_id) {
      continue;
    }
    if (new_booking.interval_.overlaps(booking.interval_)) {
      new_booking_overlaps.emplace_back(id);
      overlaps_[id].emplace_back(new_booking_id);
    }
  }
  overlaps_.emplace_back(new_booking_overlaps);
}

uint16_t heuristic_state::check_gsd_overlaps(node_id_t new_booking_id) {
  // check how many spots the mew new_booking can choose from when only
  // considering gsd bookings.
  uint16_t valid_spots = 0U;
  for (auto color = color_t{0}; color != wish_combination_by_color_.size();
       ++color) {
    if (!matches(bookings_[new_booking_id].r_,
                 wish_combination_by_color_[color])) {
      continue;
    }
    if (!is_gsd_blocked(color, bookings_[new_booking_id].interval_)) {
      ++num_valid_neighbours;
      valid_spots++;
    }
  }
  return valid_spots;
}

void heuristic_state::reset() {
  tabu_filled_ = false;
  tabu_pos_ = 0U;
}

void heuristic_state::update(neighbourhood const nh) {
  illegal_count_ += count_new_overlaps(nh.moved_, nh.new_color_);
  illegal_count_ -= count_new_overlaps(nh.moved_, nh.old_color_);
  colors_[nh.moved_] = nh.new_color_;
  if (use_tabu) {
    tabu_list_[tabu_pos_++] = nh;
    if (tabu_pos_ == tabu_list_.size()) {
      tabu_pos_ = 0U;
      tabu_filled_ = true;
    }
  }
  feasible_ = illegal_count_ == 0U ? FEASIBLE : UNKNOWN;
}

uint16_t heuristic_state::count_new_overlaps(node_id_t const moved,
                                             color_t const new_color) const {
  return std::count_if(overlaps_[moved].begin(), overlaps_[moved].end(),
                       [&](node_id_t o) { return new_color == colors_[o]; });
}

uint16_t heuristic_state::tabu_upper() const {
  return tabu_filled_ ? tabu_list_.size() : tabu_pos_;
}

int heuristic_state::get_random_int(int const& range) const {
  return rand() % range;
}

neighbourhood heuristic_state::get_random_neighbour() const {
  auto nh = neighbourhood();
  auto r = get_random_int(num_valid_neighbours - (use_tabu ? tabu_upper() : 0));
  for (auto [id, booking] : utl::enumerate(bookings_)) {
    auto shift = possible_pos_[id] - 1 - (use_tabu ? tabu_count_[id] : 0);
    r -= shift;
    if (r <= 0) {
      r += shift;
      nh.moved_ = id;
      nh.old_color_ = colors_[id];
      for (auto color = color_t{0}; color != wish_combination_by_color_.size();
           ++color) {
        if (!matches(booking.r_, wish_combination_by_color_[color]) ||
            color == colors_[id]) {
          continue;
        }
        bool found = false;
        nh.new_color_ = color;
        if (use_tabu) {
          for (auto i = 0; i != tabu_upper(); ++i) {
            if (tabu_list_[i].is_reverse(nh)) {
              found = true;
              break;
            }
          }
        }
        if (found) {
          continue;
        }
        --r;
        if (r == 0) {
          break;
        }
      }
      break;
    }
  }
  return nh;
}

double heuristic_state::score(neighbourhood const nh) const {
  auto score = double{0};
  score += count_new_overlaps(nh.moved_, nh.old_color_);
  score -= count_new_overlaps(nh.moved_, nh.new_color_);
  return score;
}

bool heuristic_state::is_gsd_blocked(color_t const color,
                                     interval const i) const {
  if (gsd_intervals_by_color_.empty()) {
    return false;
  }
  return gsd_intervals_by_color_[color].end() !=
         std::find_if(
             gsd_intervals_by_color_[color].begin(),
             gsd_intervals_by_color_[color].end(),
             [&](interval gsd_interval) { return i.overlaps(gsd_interval); });
}
}  // namespace seat