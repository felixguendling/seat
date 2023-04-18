#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

#include "utl/enumerate.h"
#include "utl/timing.h"
#include "utl/verify.h"

#include "seat/booking.h"
#include "seat/random_booking.h"
#include "seat/reservation.h"
#include "seat/simulate_booking_series.h"
#include "seat/solver.h"
#include "seat/verify_booking_assignment.h"
#include "seat/wagon_solver.h"

using namespace seat;
seat::simulation::simulation(
    std::map<reservation, std::uint32_t> const& seats_by_res, solver* solver,
    const double& gsd_prob, const double& group_prob,
    const uint32_t& number_of_segments, train const& t) {
  seats_by_res_ = seats_by_res;
  gsd_prob_ = gsd_prob;
  group_prob_ = group_prob;
  number_of_segments_ = number_of_segments;
  solver_ = solver;
  train_ = t;
}

void simulation::simulate() {
  while (failed_ > 0) {
    auto roll = static_cast<double>(rand() % 10000) / double{10000};
    if (roll < gsd_prob_) {
      add_gsd_booking();
    } else if (roll < gsd_prob_ + group_prob_) {
      add_group_booking();
    } else {
      add_normal_booking();
    }
  }

  std::cout << "\n"
            << "gsd counter: " << +gsd_counter_
            << ", non-gsd counter: " << +normal_counter_ << "\n"
            << "\n";

  uint64_t sum = 0;
  uint64_t sum_non_gsd = 0;
  uint64_t sum_gsd = 0;
  uint64_t average = 0;
  uint64_t average_gsd = 0;
  uint64_t average_non_gsd = 0;
  uint64_t max = 0;
  uint64_t q90 = 0;
  uint64_t q95 = 0;

  for (auto j = 0; j != timings_vec_.size(); ++j) {
    sum += timings_vec_[j];
    if (booking_type_[j] == GSD) {
      sum_gsd += timings_vec_[j];
    } else {
      sum_non_gsd += timings_vec_[j];
    }
  }
  average = sum / timings_vec_.size();
  average_gsd = (gsd_counter_ == 0) ? 0 : sum_gsd / gsd_counter_;
  average_non_gsd = sum_non_gsd / normal_counter_;
  max = *max_element(timings_vec_.begin(), timings_vec_.end());
  sort(begin(timings_vec_), end(timings_vec_));
  q90 = timings_vec_[static_cast<int>(timings_vec_.size() * 0.9)];
  q95 = timings_vec_[static_cast<int>(timings_vec_.size() * 0.95)];

  std::cout << "              ";
  solver_->print_name();
  std::cout << "     ";
  std::cout << "\n";
  auto print_data = std::map<std::string, uint64_t>{};
  print_data.insert(std::make_pair("sum:               ", sum));
  print_data.insert(std::make_pair("average:           ", average));
  print_data.insert(std::make_pair("average non-gsd:   ", average_non_gsd));
  print_data.insert(std::make_pair("average gsd:       ", average_gsd));
  print_data.insert(std::make_pair("sum non-gsd:       ", sum_non_gsd));
  print_data.insert(std::make_pair("sum gsd:           ", sum_gsd));
  print_data.insert(std::make_pair("max:               ", max));
  print_data.insert(std::make_pair("q90:               ", q90));
  print_data.insert(std::make_pair("q95:               ", q95));
  for (auto const& [s, v] : print_data) {
    std::cout << s << v << "\n";
  }
  std::cout << "\n";
  solver_->print_name();
  std::cout << "\n";
  solver_->print_sizes();
  std::cout << "\n";

  auto wagon_res_capacities =
      train_.get_wagon_res_capacities(solver_->gsd_seats_);
  auto wagon_solver = solver_wagon(
      number_of_segments_, wagon_res_capacities, solver_->bookings_,
      solver_->mcf_bookings_, solver_->concrete_bookings_,
      solver_->pseudo_gsd_bookings_, solver_->gsd_bookings_,
      solver_->pseudo_gsd_seats_, solver_->gsd_seats_);
  wagon_solver.create_mcf_problem();
  wagon_solver.set_hint(
      solver_->place_bookings_in_arbitrary_valid_wagons(train_));
  UTL_START_TIMING(wagon_assign_time);
  auto feas = wagon_solver.solve();
  auto const t_wagon = UTL_GET_TIMING_MS(wagon_assign_time);
  std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>> skl =
      wagon_solver.assign_seats();

  auto seat_solver = solver_seat(
      number_of_segments_, train_.get_seat_attributes(), solver_->bookings_,
      solver_->mcf_bookings_, solver_->concrete_bookings_,
      solver_->pseudo_gsd_bookings_, solver_->gsd_bookings_,
      solver_->pseudo_gsd_seats_, solver_->gsd_seats_, train_.last_seat_id_);
  seat_solver.create_mcf_problem();
  seat_solver.set_hint(
      solver_->place_bookings_on_arbitrary_valid_seats(train_));
  UTL_START_TIMING(seat_assign_time);
  auto feasible = seat_solver.solve();
  auto const t = UTL_GET_TIMING_MS(seat_assign_time);

  UTL_START_TIMING(prep_time);
  auto b_ids = std::vector<booking_id_t>();
  auto s_ids = std::vector<seat_id_t>();
  for (auto const& [id, b_id] : utl::enumerate(seat_solver.mcf_booking_ids_)) {
    for (auto seat_id = seat_id_t{0}; seat_id != seat_solver.total_seats_;
         ++seat_id) {
      if (!matches(seat_solver.bookings_[b_id].r_,
                   seat_solver.seat_attributes_[seat_id].second)) {
        continue;
      }
      if (seat_solver.is_gsd_blocked(seat_solver.bookings_[b_id].interval_,
                                     seat_id)) {
        continue;
      }
      if (seat_solver.vars_[std::make_pair(b_id, seat_id)]->solution_value() ==
          1) {
        b_ids.emplace_back(b_id);
        s_ids.emplace_back(seat_id);
      }
    }
  }
  std::pair<std::vector<booking_id_t>, std::vector<seat_id_t>> assignment =
      std::make_pair(b_ids, s_ids);
  auto const prep_t = UTL_GET_TIMING_MS(prep_time);
  std::cout << "prep time: " << prep_t << "\n";
  std::cout << "_________________________________________________\n";
  seat_solver.print_name();
  std::cout << "\n";
  seat_solver.print_sizes();
  std::cout << "time spend wagon: " << t_wagon << ", seats " << t;
  std::cout << "\n";
  std::cout << "\n";
  train_.print(number_of_segments_, assignment.first, solver_->gsd_bookings_,
               solver_->pseudo_gsd_bookings_, assignment.second,
               solver_->gsd_seats_, solver_->pseudo_gsd_seats_,
               solver_->bookings_);
  if (!feasible) {
    abort();
  }
  std::cout << "seat assignment is " << (feasible ? "" : "in") << "feasible\n";
  /*auto ver = verify(solver_->bookings_, assignment.first,
                    solver_->gsd_bookings_, solver_->pseudo_gsd_bookings_,
                    assignment.second, solver_->gsd_seats_,
                    solver_->pseudo_gsd_seats_, train_.last_seat_id_);*/
  reset();
}

void simulation::add_normal_booking() {
  normal_counter_++;
  auto const b = generate_random_booking(
      seats_by_res_, train_.get_possible_reservations(), number_of_segments_);
  std::cout << "booking: #" << success_ << " " << b << " - ";
  auto feasible = !nogo.is_nogo(b);
  if (feasible) {
    booking_type_.emplace_back(NORMAL);
    solver_->add_booking(b);
    UTL_START_TIMING(ilp);
    feasible = solver_->solve();
    auto const t = UTL_GET_TIMING_MS(ilp);
    timings_vec_.emplace_back(t);
    if (feasible) {
      success_++;
    } else {
      nogo.add_entry(b);
      solver_->remove_booking(b);
      failed_--;
    }
    std::cout << " -> " << (feasible ? "VALID" : "NOT VALID")
              << " [remaining=" << failed_ << ", success=" << success_ << "]\n";
    timings_ << i_ << "\n";
  } else {
    failed_--;
    std::cout << "nogo"
              << "\n";
  }
}

void simulation::add_gsd_booking() {
  gsd_counter_++;
  small_station_id_t from = rand() % number_of_segments_;
  small_station_id_t to = rand() % (number_of_segments_ - from) + from + 1;
  auto interv = interval{from, to};
  assert(to > from);
  assert(from >= 0);
  assert(to < number_of_segments_);
  std::cout << "gsd: #" << success_ << " " << interv << " - ";
  booking b_gsd;
  b_gsd.interval_ = interv;
  b_gsd.r_ = reservation();
  auto feasible = !gsd_nogo.is_nogo(b_gsd);
  if (!feasible) {
    failed_--;
    std::cout << "nogo - gsd"
              << "\n";
    return;
  }
  booking_type_.emplace_back(GSD);
  UTL_START_TIMING(gsd);
  std::vector<reservation> available_reservations =
      solver_->gsd_request(interv);
  timings_vec_.emplace_back(UTL_GET_TIMING_MS(gsd));
  std::vector<seat_id_t> available_seats = train_.get_available_seats(
      available_reservations, solver_->get_gsd_bookings());
  std::cout << " available seats: " << available_seats.size() << ": ";
  if (available_seats.empty()) {
    gsd_nogo.add_entry(b_gsd);
    std::cout << "gsd request denied\n";
  } else {
    success_++;
    booking b;
    // randomly choose reservation among availables
    auto s_id =
        available_seats[static_cast<int>(rand() % available_seats.size())];
    train_.get_reservation(s_id, b.r_);
    b.interval_ = interv;
    std::cout << "gsd chose seat " << s_id << ", with reservation: " << b.r_
              << "\n";
    solver_->add_gsd_booking(b, s_id);
  }
}

void simulation::add_group_booking() {
  group_counter_++;
  auto b = generate_random_booking(
      seats_by_res_, train_.get_possible_reservations(), number_of_segments_);
  uint8_t group_size = rand() % max_group_size_;
  b.group_id_ = ++last_group_id_;
  b.r_[0] = wish::kAny;
  std::cout << "group: # " << success_ << " of size " << +group_size
            << " with id " << +b.group_id_ << " " << b << " - ";
  auto feasible = !nogo.is_nogo(b);
  if (feasible) {
    booking_type_.emplace_back(NORMAL);
    for (auto i = 0; i != group_size; ++i) {
      solver_->add_booking(b);
    }
    UTL_START_TIMING(ilp);
    feasible = solver_->solve();
    auto const t = UTL_GET_TIMING_MS(ilp);
    timings_vec_.emplace_back(t);
    if (feasible) {
      success_ += group_size;
    } else {
      nogo.add_entry(b);
      for (auto i = 0; i != group_size; ++i) {
        solver_->remove_booking(b);
      }
      failed_--;
    }
    std::cout << " -> " << (feasible ? "VALID" : "NOT VALID")
              << " [remaining=" << failed_ << ", success=" << success_ << "]\n";
    timings_ << i_ << "\n";
  } else {
    failed_--;
    std::cout << "nogo"
              << "\n";
  }
}

void simulation::reset() {
  nogo.clear();
  gsd_nogo.clear();
  group_nogo.clear();
  failed_ = 1200;
  i_ = 0U;
  success_ = 0U;
  timings_vec_.clear();
  booking_type_.clear();
  gsd_counter_ = 0;
  normal_counter_ = 0;
  group_counter_ = 0;
  last_group_id_ = 0;
}
