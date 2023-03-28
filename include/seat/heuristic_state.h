#pragma once

#include <vector>

#include "seat/booking.h"
#include "seat/neighbourhood.h"
#include "seat/types.h"

namespace seat {
struct heuristic_state {
public:
  heuristic_state(std::vector<uint8_t> wish_combs, uint8_t tabus,
                  bool use_tabu = false)  // constructor definition
  {
    wish_combination_by_color_ = wish_combs;
    gsd_intervals_by_color_.resize(wish_combination_by_color_.size());
    tabu_list_.resize(tabus);
    use_tabu = use_tabu;
  }
  void reset();

  void insert_booking(booking);
  void insert_gsd_booking(color_t, interval);
  void update_overlaps(booking);
  uint16_t check_gsd_overlaps(node_id_t);
  bool move_to_best_location(node_id_t);

  void update(neighbourhood);
  int get_random_int(int const&) const;

  uint16_t count_new_overlaps(node_id_t const, color_t const) const;
  uint16_t tabu_upper() const;
  neighbourhood get_random_neighbour() const;
  double score(neighbourhood) const;
  bool is_gsd_blocked(color_t, interval) const;

  // per booking
  std::vector<booking> bookings_;
  std::vector<std::vector<node_id_t>> overlaps_;
  std::vector<color_t> colors_;
  std::vector<uint16_t> possible_pos_;
  std::vector<uint8_t> tabu_count_;

  std::vector<uint8_t> wish_combination_by_color_;

  std::vector<std::vector<interval>> gsd_intervals_by_color_;

  enum FEASIBLE feasible_ = UNKNOWN;

  uint16_t num_valid_neighbours = 0U;
  bool use_tabu;
  uint32_t tabu_pos_ = 0U;
  bool tabu_filled_ = false;
  std::vector<neighbourhood> tabu_list_;

  uint16_t illegal_count_ = uint16_t{0};
};

}  // namespace seat