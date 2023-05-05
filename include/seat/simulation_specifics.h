#pragma once

#include "seat/output_options.h"

namespace seat {
enum class final_assignment_strategy {
  FEW_GROUPS_IN_PSEUDO_WAGON_SEAT,
  DIRECT_SEAT
};
struct simulation_specifics {
  final_assignment_strategy strategy = final_assignment_strategy::DIRECT_SEAT;
  double gsd_prob_ = 0;
  double group_prob_ = 0;
  std::uint32_t number_of_segments_ = 10;
  uint8_t max_group_size_ = 10;
  uint8_t timed_wagon_runs_ = 0;
  output_options output_options_;
};

}  // namespace seat