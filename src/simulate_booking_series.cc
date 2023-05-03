#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

#include "utl/enumerate.h"
#include "utl/parser/buf_reader.h"
#include "utl/parser/csv_range.h"
#include "utl/parser/line_range.h"
#include "utl/pipes/for_each.h"
#include "utl/timing.h"
#include "utl/verify.h"

#include "cista/mmap.h"
#include "seat/booking.h"
#include "seat/random_booking.h"
#include "seat/reservation.h"
#include "seat/seat_from_wagon_assignment.h"
#include "seat/simulate_booking_series.h"
#include "seat/solver.h"
#include "seat/wagon_solver.h"

using namespace seat;
seat::simulation::simulation(
    std::map<reservation, std::uint32_t> const& seats_by_res, solver* solver,
    train const& t, simulation_specifics const& specifics) {
  specifics_ = specifics;
  seats_by_res_ = seats_by_res;
  solver_ = solver;
  train_ = t;
}

void simulation::simulate() {
  while (failed_ > 0) {
    auto roll = static_cast<double>(rand() % 10000) / double{10000};
    if (roll < specifics_.gsd_prob_) {
      add_gsd_booking();
    } else if (roll < specifics_.gsd_prob_ + specifics_.group_prob_) {
      add_group_booking();
    } else {
      add_normal_booking();
    }
  }
  write_before_wagon_assignment();
  print_valid_times();
  solver_->solve();
  // solver_->release_pseudo(train_.get_seat_attributes());
  auto hint = solver_->place_bookings_in_arbitrary_valid_wagons(train_);
  auto wagon_ids_by_booking_ids = assign_to_wagons(hint);
  print_wagon_assignment_results(wagon_ids_by_booking_ids);

  // assign bookings to seats using assignment to wagons ***
  auto seats_from_wagon_solver = solver_seat_from_wagon(
      specifics_.number_of_segments_, wagon_ids_by_booking_ids,
      train_.get_seat_attributes(), solver_->bookings_, solver_->mcf_bookings_,
      solver_->concrete_bookings_, solver_->pseudo_gsd_bookings_,
      solver_->gsd_bookings_, solver_->pseudo_gsd_seats_, solver_->gsd_seats_,
      train_.last_seat_id_, train_);
  // seats_from_wagon_solver.set_hint(wagon_ids_by_booking_ids);
  UTL_START_TIMING(wagon_to_seat);
  seats_from_wagon_solver.solve();
  auto const wagon_to_seat_t = UTL_GET_TIMING_MS(wagon_to_seat);
  //***

  UTL_START_TIMING(prep_time);
  auto const prep_t = UTL_GET_TIMING_MS(prep_time);
  std::cout << "prep time: " << prep_t << "\n";
  std::cout << "_________________________________________________\n";
  seats_from_wagon_solver.print_name();
  std::cout << "\n";
  seats_from_wagon_solver.print_sizes();
  std::cout << " wagon to seat: " << wagon_to_seat_t << ", seats "
            << wagon_to_seat_t;
  std::cout << "\n";
  std::cout << "\n";
  train_.print2(specifics_.number_of_segments_, solver_->bookings_,
                seats_from_wagon_solver.seats_by_bookings_,
                solver_->gsd_bookings_, solver_->gsd_seats_);
  reset();
}

void simulation::add_normal_booking() {
  normal_counter_++;
  auto b =
      generate_random_booking(seats_by_res_, train_.get_possible_reservations(),
                              specifics_.number_of_segments_);
  b.type_ = booking_type::NORMAL;
  std::cout << "booking: #" << success_ << " " << b << " - ";
  auto feasible = !nogo.is_nogo(b);
  if (feasible) {
    booking_type_.emplace_back(booking_type::NORMAL);
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
  small_station_id_t from = rand() % specifics_.number_of_segments_;
  small_station_id_t to =
      rand() % (specifics_.number_of_segments_ - from) + from + 1;
  auto interv = interval{from, to};
  assert(to > from);
  assert(from >= 0);
  assert(to < specifics_.number_of_segments_);
  std::cout << "gsd: #" << success_ << " " << interv << " - ";
  booking b_gsd;
  b_gsd.type_ = booking_type::GSD;
  b_gsd.interval_ = interv;
  b_gsd.r_ = reservation();
  auto feasible = !gsd_nogo.is_nogo(b_gsd);
  if (!feasible) {
    failed_--;
    std::cout << "nogo - gsd"
              << "\n";
    return;
  }
  booking_type_.emplace_back(booking_type::GSD);
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
  auto b =
      generate_random_booking(seats_by_res_, train_.get_possible_reservations(),
                              specifics_.number_of_segments_);
  b.type_ = booking_type::GROUP;
  uint8_t g_s1 = (rand() % specifics_.max_group_size_);
  uint8_t g_s2 = (rand() % specifics_.max_group_size_);
  uint8_t group_size = (g_s1 > g_s2) ? (g_s2 + 2) : (g_s1 + 2);
  b.group_id_ = last_group_id_;
  b.r_[0] = wish::kAny;
  std::cout << "group  : #" << success_ << " of size " << +group_size
            << " with id " << +b.group_id_ << " " << b << " - ";
  auto feasible = !nogo.is_nogo(b);
  if (feasible) {
    booking_type_.emplace_back(booking_type::GROUP);
    for (auto i = 0; i != group_size; ++i) {
      solver_->add_booking(b);
    }
    UTL_START_TIMING(ilp);
    feasible = solver_->solve();
    auto const t = UTL_GET_TIMING_MS(ilp);
    timings_vec_.emplace_back(t);
    if (feasible) {
      success_ += group_size;
      ++last_group_id_;
    } else {
      // nogo.add_entry(b);
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

std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>>
simulation::assign_to_wagons(std::vector<wagon_id_t> const& hint) {
  auto wagon_res_capacities =
      train_.get_wagon_res_capacities(std::vector<booking_id_t>()); /*
   auto errors = std::vector<int>();
   auto times = std::vector<int>();
   errors.resize(specifics_.timed_wagon_runs_);
   times.resize(specifics_.timed_wagon_runs_);
   std::fill(errors.begin(), errors.end(), 0);
    for (auto run_num = 1; run_num != specifics_.timed_wagon_runs_ + 1;
         ++run_num) {
      auto wagon_solver = solver_wagon(
          specifics_.number_of_segments_, wagon_res_capacities,
          solver_->bookings_, solver_->mcf_bookings_,
    solver_->concrete_bookings_, solver_->pseudo_gsd_bookings_,
    solver_->gsd_bookings_, solver_->pseudo_gsd_seats_, solver_->gsd_seats_);
      wagon_solver.create_mcf_problem();
      wagon_solver.set_hint(
          solver_->place_bookings_in_arbitrary_valid_wagons(train_));
      wagon_solver.create_objective(train_.train_.size());
      UTL_START_TIMING(wagon_assign_time);
      wagon_solver.solve(run_num);
      auto const t_wagon = UTL_GET_TIMING_MS(wagon_assign_time);
      std::cout << "----------\n";
      errors[run_num - 1] = wagon_solver.print_helpers(false);
      times[run_num - 1] = t_wagon;
      wagon_solver.reset();
      if (errors[run_num - 1] == 0) {
        break;
      }
    }
    for (auto [idx, err] : utl::enumerate(errors)) {
      std::cout << "time limit: " << +idx + 1 << ",     errors: " << +err
                << ",   time spend: " << times[idx] << "\n";
    }*/
  auto wagon_solver = solver_wagon(
      specifics_.number_of_segments_, wagon_res_capacities, solver_->bookings_,
      solver_->mcf_bookings_, solver_->concrete_bookings_,
      solver_->pseudo_gsd_bookings_, solver_->gsd_bookings_,
      solver_->pseudo_gsd_seats_, solver_->gsd_seats_);
  wagon_solver.create_mcf_problem(train_);
  wagon_solver.set_hint(hint);
  wagon_solver.create_objective(train_.train_.size());
  wagon_solver.print();
  UTL_START_TIMING(wagon_assign_time);
  wagon_solver.solve(4);
  auto const t_wagon = UTL_GET_TIMING_MS(wagon_assign_time);
  std::cout << "----------\n";
  std::cout << "time spend wagon: " << t_wagon << "\n";
  std::cout << "errors: " << wagon_solver.print_helpers(false) << "\n";
  return wagon_solver.assign_seats();
}

void simulation::print_valid_times() {
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
    if (booking_type_[j] == booking_type::GSD) {
      sum_gsd += timings_vec_[j];
    } else {
      sum_non_gsd += timings_vec_[j];
    }
  }
  average = sum / timings_vec_.size();
  average_gsd = (gsd_counter_ == 0) ? 0 : sum_gsd / gsd_counter_;
  average_non_gsd = (normal_counter_ == 0) ? 0 : sum_non_gsd / normal_counter_;
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
  auto mcf_count = solver_->mcf_bookings_.size();
  auto max_group_idd = 0;
  for (auto const& b : solver_->bookings_) {
    if (b.group_id_ > max_group_idd) {
      max_group_idd = b.group_id_;
    }
  }
  auto gsd_count = solver_->gsd_bookings_.size();
  auto pseudo_count = solver_->pseudo_gsd_bookings_.size();
  std::cout << "-------------------\n"
            << "mcf: " << mcf_count << "\ngsd: " << gsd_count
            << "\npseudo: " << pseudo_count << "\ngroups: " << max_group_idd
            << "\n";
}

void simulation::print_wagon_assignment_results(
    std::pair<std::vector<booking_id_t>, std::vector<wagon_id_t>> const&
        wagon_ids_by_booking_ids) const {
  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
  auto max_group_id = -1;
  for (auto const& idx : wagon_ids_by_booking_ids.first) {
    auto g_id = solver_->bookings_[idx].group_id_;
    if (static_cast<int>(g_id) > max_group_id) {
      max_group_id = g_id;
    }
  }
  if (max_group_id > 0) {
    for (auto ij = 1; ij != max_group_id + 1; ++ij) {
      std::cout << "group id: " << ij << "   ";
      for (auto const& [idx, b_id] :
           utl::enumerate(wagon_ids_by_booking_ids.first)) {
        if (solver_->bookings_[b_id].group_id_ == ij) {
          SetConsoleTextAttribute(hConsole,
                                  solver_->bookings_[b_id].group_id_ % 15);
          std::cout << "(" << b_id << ", "
                    << +wagon_ids_by_booking_ids.second[idx] << ")    ";
          SetConsoleTextAttribute(hConsole, 15);
        }
      }
      std::cout << "\n";
    }
  }
  SetConsoleTextAttribute(hConsole, 15);
}

void simulation::reset() {
  nogo.clear();
  gsd_nogo.clear();
  group_nogo.clear();
  failed_ = 420;
  i_ = 0U;
  success_ = 0U;
  timings_vec_.clear();
  booking_type_.clear();
  gsd_counter_ = 0;
  normal_counter_ = 0;
  group_counter_ = 0;
  last_group_id_ = 1;
}

void simulation::write_before_wagon_assignment() {
  auto const file_name = "before_wagons.csv";
  auto f = std::ofstream{file_name};
  f << "from;to;reservation;type;group_size;group_id";
  for (auto const& b : solver_->bookings_) {
    f << b.interval_.from_ << ";" << b.interval_.to_ << ";" << b.r_ << ";"
      << b.type_ << ";" << b.group_size_ << ";" << b.group_id_ << "\n";
  }
}

void simulation::read_bookings(std::string const& file_name) {
  struct entry {
    utl::csv_col<small_station_id_t, UTL_NAME("from")> from_;
    utl::csv_col<small_station_id_t, UTL_NAME("to")> to_;
    utl::csv_col<utl::cstr, UTL_NAME("reservation")> r_;
    utl::csv_col<utl::cstr, UTL_NAME("type")> type_;
    utl::csv_col<uint8_t, UTL_NAME("group_size")> g_size_;
    utl::csv_col<group_id_t, UTL_NAME("group_id")> g_id_;
  };
  auto m = cista::mmap{file_name.c_str(), cista::mmap::protection::READ};

  utl::line_range{utl::buf_reader{m.view()}}  //
      | utl::csv<entry>()  //
      | utl::for_each([&](entry const& e) {
          booking b;
          b.interval_ = interval{e.from_.val(), e.to_.val()};
          b.group_id_ = e.g_id_.val();
          b.group_size_ = e.g_size_.val();
        });
}
