#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

#include "utl/timing.h"
#include "utl/verify.h"

#include "seat/booking.h"
#include "seat/nogo_cache.h"
#include "seat/random_booking.h"
#include "seat/reservation.h"
#include "seat/simulate_booking_series.h"
#include "seat/solver.h"

using namespace seat;
seat::simulation::simulation(
    std::map<reservation, std::uint32_t> const& seats_by_res,
    const std::vector<mcf_solver_i*>& solvers, const double& gsd_prob,
    const uint32_t& number_of_segments, train const& t) {
  seats_by_res_ = seats_by_res;
  gsd_prob_ = gsd_prob;
  number_of_segments_ = number_of_segments;
  solvers_ = solvers;
  train_ = t;
}

void simulation::simulate() {
  auto failed = 1000;
  nogo_cache nogo;

  auto i = 0U;
  auto success = 0U;

  auto timings = std::ofstream{"timings.dat"};
  std::vector<std::vector<std::uint64_t>> timings_vec;
  timings_vec.resize(solvers_.size());
  std::vector<bool> gsd;
  auto gsd_c = 0;
  auto pr_c = 0;

  auto feasible_vec = std::vector<bool>{};
  feasible_vec.resize(solvers_.size());
  while (failed > 0) {
    if ((rand() / static_cast<double>(RAND_MAX) > gsd_prob_)) {
      pr_c++;
      auto const b = generate_random_booking(seats_by_res_,
                                             train_.get_possible_reservations(),
                                             number_of_segments_);
      std::cout << "adding booking: #" << ++i << " " << b << " - ";
      auto feasible = !nogo.is_nogo(b);
      if (feasible) {
        auto fake = false;
        gsd.emplace_back(false);
        for (auto j = 0; j != solvers_.size(); ++j) {
          solvers_[j]->add_booking(b);
          UTL_START_TIMING(ilp);
          feasible_vec[j] = solvers_[j]->solve();
          auto const t = UTL_GET_TIMING_MS(ilp);
          timings_vec[j].emplace_back(t);
          if (!feasible_vec[j]) {
            feasible = false;
          }
          if (j > 0 && (feasible_vec[j] != feasible_vec[j - 1])) {
            fake = true;
          }
        }
        if (fake) {
          solvers_[0]->print_name();
          solvers_[0]->print();
          solvers_[1]->print_name();
          solvers_[1]->print(); /*
           solvers_[2]->print_name();
           solvers_[2]->print();
           */
          abort();
        }
        if (!feasible) {
          nogo.add_entry(b);
          for (auto& solver : solvers_) {
            solver->remove_booking(b);
          }
          failed--;
        }
        if (feasible) {
          success++;
        }
        std::cout << " -> " << (feasible ? "VALID" : "NOT VALID")
                  << " [remaining=" << failed << ", success=" << success
                  << "]\n";
        timings << i << "\n";
      } else {
        failed--;
        std::cout << "nogo"
                  << "\n";
      }
    } else {
      gsd.emplace_back(true);
      gsd_c++;
      small_station_id_t from = rand() % number_of_segments_;
      small_station_id_t to = rand() * (number_of_segments_ - from - 1) + from;
      auto interv = interval{from, to};
      assert(to >= from);
      assert(from >= 0);
      assert(to < number_of_segments_);
    }
  }
  std::cout << "\n"
            << "gsd counter: " << +gsd_c << ", non-gsd counter: " << +pr_c
            << "\n"
            << "\n";

  std::vector<uint64_t> sum;
  std::vector<uint64_t> sum_non_gsd;
  std::vector<uint64_t> sum_gsd;
  std::vector<uint64_t> average;
  std::vector<uint64_t> max;
  std::vector<uint64_t> q90;
  std::vector<uint64_t> q95;
  sum.resize(solvers_.size());
  sum_non_gsd.resize(solvers_.size());
  sum_gsd.resize(solvers_.size());
  average.resize(solvers_.size());
  max.resize(solvers_.size());
  q90.resize(solvers_.size());
  q95.resize(solvers_.size());
  std::fill(sum.begin(), sum.end(), 0);
  std::fill(sum_non_gsd.begin(), sum_non_gsd.end(), 0);
  std::fill(sum_gsd.begin(), sum_gsd.end(), 0);
  std::fill(average.begin(), average.end(), 0);
  std::fill(max.begin(), max.end(), 0);
  std::fill(q90.begin(), q90.end(), 0);
  std::fill(q95.begin(), q95.end(), 0);

  for (auto k = 0; k != solvers_.size(); ++k) {
    for (auto j = 0; j != timings_vec[k].size(); ++j) {
      sum[k] += timings_vec[k][j];
      if (gsd[j]) {
        sum_gsd[k] += timings_vec[k][j];
      } else {
        sum_non_gsd[k] += timings_vec[k][j];
      }
    }
    average[k] = sum[k] / timings_vec[k].size();
    max[k] = *max_element(timings_vec[k].begin(), timings_vec[k].end());
    sort(begin(timings_vec[k]), end(timings_vec[k]));
    q90[k] = timings_vec[k][static_cast<int>(timings_vec[k].size() * 0.9)];
    q95[k] = timings_vec[k][static_cast<int>(timings_vec[k].size() * 0.95)];
  }
  std::cout << "              ";
  for (auto k = 0; k != solvers_.size(); ++k) {
    solvers_[k]->print_name();
    std::cout << "     ";
  }
  std::cout << "\n";
  auto print_data = std::map<std::string, std::vector<uint64_t>>{};
  print_data.insert(std::make_pair("sum:               ", sum));
  print_data.insert(std::make_pair("average:           ", average));
  print_data.insert(std::make_pair("sum non-gsd:       ", sum_non_gsd));
  print_data.insert(std::make_pair("sum gsd:           ", sum_gsd));
  print_data.insert(std::make_pair("max:               ", max));
  print_data.insert(std::make_pair("q90:               ", q90));
  print_data.insert(std::make_pair("q95:               ", q95));
  for (auto const& [s, v] : print_data) {
    std::cout << s;
    for (auto const& data : v) {
      std::cout << data << "                  ";
    }
    std::cout << "\n";
  }
  std::cout << "\n";
  for (auto const& s : solvers_) {
    s->print_name();
    std::cout << "\n";
    s->print_sizes();
    std::cout << "\n";
  }

  auto seat_solver =
      solver_seat(number_of_segments_, train_.get_seat_attributes(),
                  solvers_[0]->get_mcf_bookings(),
                  solvers_[0]->get_gsd_bookings(), train_.last_seat_id_);
  seat_solver.create_mcf_problem();
  UTL_START_TIMING(seat_assign_time);
  auto feasible = seat_solver.solve();
  auto const t = UTL_GET_TIMING_MS(seat_assign_time);

  UTL_START_TIMING(prep_time);
  cista::raw::vector_map<station_id_t,
                         std::map<seat_id_t, std::pair<booking_id_t, booking>>>
      assignment = seat_solver.assign_seats();
  auto const prep_t = UTL_GET_TIMING_MS(prep_time);
  std::cout << "prep time: " << prep_t << "\n";
  /*cista::raw::vector_map<booking_id_t, booking> bss =
      solvers_[0]->get_mcf_bookings();
  for (auto const& [s, b] : utl::enumerate(bss)) {
    std::cout << "id: " << s << ", interval: " << b.interval_
              << ", reservation: " << b.r_ << "\n";
  }
  for (auto const& [station, booking_by_seat] : utl::enumerate(assignment)) {
    std::cout << "station: " << station << "----------\n";
    for (auto const& [seat, booking] : booking_by_seat) {
      std::cout << "seat: " << seat << ", booking: " << booking.first << "\n";
    }
  }*/
  std::cout << "_________________________________________________\n";
  seat_solver.print_name();
  std::cout << "\n";
  seat_solver.print_sizes();
  std::cout << "time spend: " << t;
  std::cout << "\n";
  std::cout << "\n";
  train_.print(assignment);
  std::cout << "seat assignment is " << (feasible ? "" : "in") << "feasible";
}