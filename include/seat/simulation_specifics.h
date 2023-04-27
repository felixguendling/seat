#pragma once

#include "seat/output_options.h"

namespace seat {
struct simulation_specifics {
  double gsd_prob_ = 0;
  double group_prob_ = 0;
  std::uint32_t number_of_segments_ = 10;
  uint8_t max_group_size_ = 10;
  uint8_t timed_wagon_runs_ = 0;
  output_options output_options_;
};

}  // namespace seat