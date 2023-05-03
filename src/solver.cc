#include "seat/solver.h"

#include "utl/get_or_create.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"
#include "utl/enumerate.h"

#include "seat/interval_graph.h"
#include "seat/pseudo_gsd_chooser.h"
#include "seat/train.h"

namespace seat {

constexpr auto const solver_name = "SCIP";
solver::solver(std::map<reservation, uint32_t> const& capacities,
               uint32_t const number_segments)
    : solver_{gor::MPSolver::CreateSolver(solver_name)},
      number_segments_{number_segments} {
  for (auto const& [r, c] : capacities) {
    if (c > 0) {
      c_.emplace_back(r);
    }
    for (auto i = small_station_id_t{0}; i != number_segments; ++i) {
      auto key = std::make_pair(r, i);
      capacities_.emplace(key, c);
    }
  }
}

bool solver::solve() {
  if (invalid) {
    invalid = false;
    return false;
  }
  clear();
  for (auto const& id : mcf_bookings_) {
    create_vars_and_constraints(id);
  }
  replace_ones();
  result_ = solver_->Solve();
  return feasible();
};

bool solver::feasible() {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver::add_booking(booking const& new_b) {
  bookings_.emplace_back(new_b);
  if (concreteness(new_b.r_) == 0) {
    concrete_booking_adjust(new_b, true);
  } else {
    mcf_bookings_.emplace_back(bookings_.size() - 1);
  }
}

void solver::erase(std::vector<booking_id_t>& vec, booking const& b) {
  auto idx_in_bookings = booking_id_t{0};
  for (auto i = vec.size() - 1; true; --i) {
    if (bookings_[vec[i]] == b) {
      idx_in_bookings = vec[i];
      bookings_.erase(std::next(bookings_.begin(), idx_in_bookings));
      break;
    }
  }
  auto const adjust_positions = [&](std::vector<booking_id_t>& v) {
    for (auto j = 0; j != v.size(); ++j) {
      if (v[j] <= idx_in_bookings) {
        continue;
      }
      v[j]--;
    }
  };
  // update the indexes in the permutation vector to account for the deleted
  // entry in bookings
  adjust_positions(mcf_bookings_);
  adjust_positions(concrete_bookings_);
  adjust_positions(gsd_bookings_);
  adjust_positions(pseudo_gsd_bookings_);
  vec.erase(std::find(begin(vec), end(vec), idx_in_bookings));
}

void solver::concrete_booking_adjust(booking const& b, bool const added) {
  if (added) {
    concrete_bookings_.emplace_back(bookings_.size() - 1);
  } else {
    erase(concrete_bookings_, b);
  }
  for (auto i = b.interval_.from_; i != b.interval_.to_; ++i) {
    auto pair = std::make_pair(b.r_, i);
    if (added) {
      capacities_[pair]--;
    } else {
      capacities_[pair]++;
    }
    if (capacities_[std::make_pair(b.r_, i)] < 0) {
      invalid = true;
    }
  }
}

void solver::remove_booking(booking const& rem_b) {
  if (concreteness(rem_b.r_) == 0) {
    concrete_booking_adjust(rem_b, false);
    return;
  }
  auto bb = rem_b;
  erase(mcf_bookings_, bb);
}

void solver::create_vars_and_constraints(booking_id_t const& new_b) {
  auto b = bookings_[new_b];
  auto from = b.interval_.from_;
  auto const to = b.interval_.to_;
  auto source_constraint = get_source_constraint(bookings_[new_b]);
  if (output_) {
    std::cout << "\n";
    std::cout << bookings_[new_b] << ":  " << to_int_all(bookings_[new_b].r_)
              << "\n";
  }
  check_initial_bounds(source_constraint);
  source_constraint->SetBounds(source_constraint->lb() + 1,
                               source_constraint->ub() + 1);
  std::vector<reservation> concrete_reservations =
      get_concrete_reservations(bookings_[new_b].r_);
  for (auto const& concrete_r : concrete_reservations) {
    if (std::find(c_.begin(), c_.end(), concrete_r) == c_.end()) {
      continue;
    }
    from = b.interval_.from_;
    auto source_var = get_path_var(bookings_[new_b], concrete_r);
    source_var->SetUB(source_var->ub() + one_);
    source_constraint->SetCoefficient(source_var, 1);
    while (from != to) {
      auto capacity_constraint = get_capacity_constraint(from, concrete_r);
      capacity_constraint->SetCoefficient(source_var, 1);
      auto key = std::make_pair(concrete_r, from);
      capacity_constraint->SetBounds(0.0, capacities_[key]);
      ++from;
    }
  }
}

gor::MPVariable* solver::get_path_var(booking const& b,
                                      reservation const& c_res) {
  auto const i = b.interval_;
  auto pair = std::make_pair(c_res, b);
  return utl::get_or_create(path_vars_, pair, [&]() {
    return solver_->MakeIntVar(
        0.0, 0.0,
        fmt::format("source_{}_{}_{}_{}", b.r_, i.from_, i.to_, c_res));
  });
}

gor::MPConstraint* solver::get_source_constraint(booking const& b) {
  auto const i = b.interval_;
  return utl::get_or_create(source_constraints_, b, [&]() {
    return solver_->MakeRowConstraint(
        fmt::format("source_c_{}_{}_{}", b.r_, i.from_, i.to_));
  });
}

gor::MPConstraint* solver::get_capacity_constraint(
    small_station_id_t const from, reservation const& res) {
  return utl::get_or_create(capacity_constraints_, std::make_pair(res, from),
                            [&]() {
                              return solver_->MakeRowConstraint(
                                  fmt::format("capacity_c_{}_{}", res, from));
                            });
}

// workaround for {0,1}-int-vars become bool-vars in ortools internally
void solver::replace_ones() {
  for (const auto [key, var] : path_vars_) {
    if ((static_cast<int>(var->ub()) % one_) == 0) {
      var->SetUB(static_cast<int>(var->ub()) / one_);
    }
  }
}

void solver::clear() {
  solver_->Reset();
  solver_ = gor::MPSolver::CreateSolver(solver_name);
  capacity_constraints_.clear();
  source_constraints_.clear();
  path_vars_.clear();
}
void solver::check_initial_bounds(gor::MPConstraint* c) {
  if (c->ub() == gor::MPSolver::infinity()) {
    c->SetUB(0.0);
  }
  if (c->lb() == -gor::MPSolver::infinity()) {
    c->SetLB(0.0);
  }
}

void solver::print() const {
  std::string s;
  solver_->ExportModelAsLpFormat(false, &s);
  std::cout << s;
}

void solver::print_sizes() const {
  std::cout << "vars: " << solver_->NumVariables() << "\n"
            << "constraints: " << solver_->NumConstraints()
            << "\n"; /*
std::cout << "capacity_constraints_: " << capacity_constraints_.size() << "\n"
<< "source_constraints_: " << source_constraints_.size() << "\n"
<< "path_vars_: " << path_vars_.size() << "\n";*/
}

void solver::print_name() const { std::cout << "full path solver"; }

std::vector<reservation> solver::gsd_request(interval const& i) {
  // determine which seats are available for a gsd-booking with the given
  // interval i by solving mcf problems that result by adding a booking with
  // interval i and every possible concrete reservation (could run in parallel)

  // find all concrete reservations, could be done more efficiently..
  std::vector<reservation> available_res;
  auto first_station_id = small_station_id_t{0};
  auto res_counter = 0U;
  std::vector<reservation> concrete_reservations;
  for (auto const& [key, station] : capacities_) {
    if (key.second != first_station_id) {
      continue;
    }
    concrete_reservations.emplace_back(key.first);
  }

  // solve the mcf problems for every concrete reservation
  for (auto const& reservation : concrete_reservations) {
    auto b = booking{};
    b.r_ = reservation;
    b.interval_ = i;
    add_booking(b);
    if (solve()) {
      available_res.emplace_back(reservation);
    }
    remove_booking(b);
  }
  return available_res;
}

void solver::add_gsd_booking(booking const& gsd_booking,
                             seat_id_t const& seat_number) {
  // find the pseudo-gsd-bookings that go on the same seat as the current
  // gsd-booking

  // step 1: solve the mcf-problem with the chosen gsd-seat and gsd-booking NOT
  // removed
  add_booking(gsd_booking);
  solve();
  remove_booking(gsd_booking);
  bookings_.emplace_back(gsd_booking);
  gsd_bookings_.emplace_back(bookings_.size() - 1);
  gsd_seats_.emplace_back(seat_number);
  // step 2: the solution of this mcf yields an assignment of all bookings to
  //        concrete reservations
  //        -> use this to determine the set S of all segments for which the
  //        concrete reservation of the gsd-booking is at capacity and which are
  //        not covered by the interval of the gsd-booking as well as the set B
  //        of all bookings that might be put on the gsd-seat
  std::vector<small_station_id_t> tight_capacities =
      find_tight_capacities(gsd_booking);
  if (tight_capacities.empty()) {
    for (auto i = 0; i != number_segments_; ++i) {
      capacities_[std::make_pair(gsd_booking.r_, i)]--;
    }
    return;
  }
  std::cout << "\n tight capacities:\n";
  for (auto const tc : tight_capacities) {
    std::cout << +tc << ", ";
  }
  std::cout << "\n";
  auto sorted_bookings = sort_bookings_into_res(gsd_booking.r_);
  std::vector<booking_id_t> valid_bookings;
  /*
  for (auto const& [id, pair] : utl::enumerate(sorted_bookings)) {
    auto interv = pair.first.interval_;
    if (interv.from_ > gsd_booking.interval_.to_ &&
        gsd_booking.interval_.from_ > interv.to_) {
      continue;
    }
    auto counter = pair.second;
    auto pos = std::find_if(bookings_.begin(), bookings_.end(),
                            [&](booking b) { return b == pair.first; });
    do {
      if (std::find(mcf_bookings_.begin(), mcf_bookings_.end(),
                    pos - bookings_.begin()) == mcf_bookings_.end() &&
          std::find(concrete_bookings_.begin(), concrete_bookings_.end(),
                    pos - bookings_.begin()) == concrete_bookings_.end()) {
        pos = std::next(pos);
        continue;
      }
      valid_bookings.emplace_back(pos - bookings_.begin());
      pos = std::next(pos);
      counter--;
      pos = std::find_if(pos, bookings_.end(),
                         [&](booking b) { return b == pair.first; });
    } while (counter > 0);
  }*/
  for (auto const& pair : sorted_bookings) {
    auto interv = pair.first.interval_;
    if ((interv.from_ >= gsd_booking.interval_.from_ &&
         interv.from_ < gsd_booking.interval_.to_) ||
        (gsd_booking.interval_.from_ >= interv.from_ &&
         gsd_booking.interval_.from_ < interv.to_)) {
      continue;
    }
    auto counter = pair.second;
    for (auto const& id : mcf_bookings_) {
      if (counter == 0) {
        break;
      }
      if (bookings_[id] == pair.first) {
        valid_bookings.emplace_back(id);
        counter--;
      }
    }
  }

  for (auto const& id : concrete_bookings_) {
    if (bookings_[id].interval_.overlaps(gsd_booking.interval_)) {
      continue;
    }
    if (!matches(bookings_[id].r_, gsd_booking.r_)) {
      continue;
    }
    valid_bookings.emplace_back(id);
  }
  // step 3: solve an optimization problem which finds a set B of bookings
  // such
  //         that every segment in S is covered by some booking in B and such
  //         that no two bookings overlap with each other or the gsd-booking.
  //         The set B is the set of bookings that will share the gsd-seat.
  //         If this is solved fast, it is possible to use an objective
  //         function to maximize the amount of segments that are filled by B
  //         or ensure that B uses bookings with long intervals or avoids
  //         groups etc.
  auto bla = std::find(begin(bookings_), end(bookings_), gsd_booking) -
             begin(bookings_);
  auto chooser = pseudo_gsd_chooser(valid_bookings, bookings_, number_segments_,
                                    gsd_booking.interval_, tight_capacities);
  std::cout << "\ngsd booking: " << gsd_booking << "id: " << bla << "\n";
  std::cout << "seat number: " << seat_number << "\n";
  for (auto const& cb : chooser.chosen_bookings_) {
    booking b = bookings_[cb];
    pseudo_gsd_bookings_.emplace_back(cb);
    pseudo_gsd_seats_.emplace_back(seat_number);
    std::cout << "     pseudo booking: " << b << " id: " << cb << "\n";
    if ((concreteness(b.r_) == 0)) {
      concrete_bookings_.erase(
          std::find(concrete_bookings_.begin(), concrete_bookings_.end(), cb));
      for (auto i = b.interval_.from_; i != b.interval_.to_; ++i) {
        capacities_[std::make_pair(b.r_, i)]++;
      }
    } else {
      mcf_bookings_.erase(
          std::find(mcf_bookings_.begin(), mcf_bookings_.end(), cb));
    }
  }
  for (auto i = 0; i != number_segments_; ++i) {
    capacities_[std::make_pair(gsd_booking.r_, i)]--;
  }
}

std::vector<small_station_id_t> solver::find_tight_capacities(
    booking const& gsd_b) const {
  auto tight_capacities = std::vector<small_station_id_t>();
  for (auto const& [constraint_key, constraint] : capacity_constraints_) {
    auto const segment = constraint_key.second;
    if (gsd_b.interval_.from_ <= segment && gsd_b.interval_.to_ > segment) {
      continue;
    }
    // check only those constraints associated with the concrete reservation
    // combination of the gsd-booking
    if (!matches(constraint_key.first, gsd_b.r_)) {
      continue;
    }
    // count how many bookings have been chosen to occupy seats on the segment
    // associated with the capacity-constraint
    auto c = 0U;

    for (auto const& [var_key, var] : path_vars_) {
      if (constraint->GetCoefficient(var) > 0) {
        c += var->solution_value();
      }
    }
    // if the amount of occupied seats matches the capacity, it is considered
    // tight-> add the segment
    if (constraint->ub() == c) {
      tight_capacities.emplace_back(segment);
    }
  }
  for (auto const& [pair, counter] : capacities_) {
    reservation r = pair.first;
    if (!matches(r, gsd_b.r_)) {
      continue;
    }
    if (counter == 0) {
      tight_capacities.emplace_back(pair.second);
    }
  }
  return tight_capacities;
}

std::vector<seat_id_t> solver::place_bookings_on_arbitrary_valid_seats(
    train& train) {
  auto const concrete_res =
      get_concrete_reservations(train.get_possible_reservations());
  auto interv_graph = interval_graph(bookings_);
  auto seats_by_booking_id = std::vector<seat_id_t>();
  seats_by_booking_id.resize(bookings_.size());
  fill(seats_by_booking_id.begin(), seats_by_booking_id.end(),
       std::numeric_limits<seat_id_t>::max());
  auto const place_gsd = [&](std::vector<booking_id_t> v1,
                             std::vector<seat_id_t> v2) {
    for (auto const& [id, b_id] : utl::enumerate(v1)) {
      seats_by_booking_id[b_id] = v2[id];
    }
  };
  place_gsd(gsd_bookings_, gsd_seats_);
  place_gsd(pseudo_gsd_bookings_, pseudo_gsd_seats_);
  std::vector<booking_id_t> valid_bookings;
  std::vector<booking_id_t> valid_gsd;
  for (auto const& r : concrete_res) {
    valid_bookings.clear();
    get_actual_booking_ids_from_amount(sort_bookings_into_res(r),
                                       valid_bookings, r, seats_by_booking_id);
    interv_graph.create_graph(valid_bookings);
    std::vector<seat_id_t> seats = train.get_seats(r);
    for (auto const& [idx, b_id] : utl::enumerate(interv_graph.b_ids_)) {
      if (idx == seats_by_booking_id.size()) {
        break;
      }
      seats_by_booking_id[b_id] = seats[interv_graph.seats_by_booking_id_[idx]];
    }
  }
  return seats_by_booking_id;
}

void solver::get_actual_booking_ids_from_amount(
    std::vector<std::pair<booking, int>> const& amount_by_booking,
    std::vector<booking_id_t>& container, reservation const& r,
    std::vector<seat_id_t> const& seats_by_booking_id) const {
  for (auto const& [id, pair] : utl::enumerate(amount_by_booking)) {
    auto counter = pair.second;
    auto pos = std::find_if(bookings_.begin(), bookings_.end(),
                            [&](booking b) { return b == pair.first; });
    do {
      if (std::find(mcf_bookings_.begin(), mcf_bookings_.end(),
                    pos - bookings_.begin()) == mcf_bookings_.end() &&
          std::find(concrete_bookings_.begin(), concrete_bookings_.end(),
                    pos - bookings_.begin()) == concrete_bookings_.end()) {
        pos = std::next(pos);
        pos = std::find_if(pos, bookings_.end(),
                           [&](booking b) { return b == pair.first; });
        continue;
      }
      if (seats_by_booking_id[pos - bookings_.begin()] !=
          std::numeric_limits<seat_id_t>::max()) {
        pos = std::next(pos);
        pos = std::find_if(pos, bookings_.end(),
                           [&](booking b) { return b == pair.first; });
        continue;
      }
      container.emplace_back(pos - bookings_.begin());
      pos = std::next(pos);
      counter--;
      pos = std::find_if(pos, bookings_.end(),
                         [&](booking b) { return b == pair.first; });
    } while (counter > 0);
  }
  for (auto const& id : concrete_bookings_) {
    if (!matches(bookings_[id].r_, r)) {
      continue;
    }
    container.emplace_back(id);
  }
}

std::vector<std::pair<booking, int>> solver::sort_bookings_into_res(
    reservation const& r) const {
  std::vector<std::pair<booking, int>> sorted_bookings;
  for (auto const& [key, var] : path_vars_) {
    if (key.first != r) {
      continue;
    }
    auto const amount = var->solution_value();
    if (amount == 0) {
      continue;
    }
    auto const b = key.second;
    sorted_bookings.emplace_back(std::make_pair(b, amount));
  }
  return sorted_bookings;
}

std::vector<booking> solver::get_mcf_bookings() {
  auto bookings = std::vector<booking>{};
  for (auto const& id : mcf_bookings_) {
    bookings.emplace_back(bookings_[id]);
  }
  for (auto const& id : concrete_bookings_) {
    bookings.emplace_back(bookings_[id]);
  }
  return bookings;
}

std::map<seat_id_t, std::vector<booking>> solver::get_gsd_bookings() {
  std::map<seat_id_t, std::vector<booking>> gsd_bookings;

  auto const insert_booking = [&](std::vector<seat_id_t> const& seats,
                                  std::vector<booking_id_t> const& bookings) {
    for (auto const& [id, booking_id] : utl::enumerate(bookings)) {
      auto v = utl::get_or_create(gsd_bookings, seats[id],
                                  []() { return std::vector<booking>(); });
      auto b = bookings_[booking_id];
      v.emplace_back(b);
    }
  };

  insert_booking(gsd_seats_, gsd_bookings_);
  insert_booking(pseudo_gsd_seats_, pseudo_gsd_bookings_);
  return gsd_bookings;
}

std::vector<wagon_id_t> solver::place_bookings_in_arbitrary_valid_wagons(
    train& t) {
  auto seats = place_bookings_on_arbitrary_valid_seats(t);
  auto wagons = std::vector<wagon_id_t>();
  wagons.resize(seats.size());
  for (auto const& [idx, s_id] : utl::enumerate(seats)) {
    wagons[idx] = t.seat_id_to_wagon_id(s_id);
  }
  return wagons;
}

void solver::to_graphviz(std::ostream&, bool const) const {}

void solver::release_pseudo(
    std::map<seat_id_t, std::pair<wagon_id_t, reservation>> const&
        seat_attributes) {
  auto insert_sorted = [](std::vector<booking_id_t>& v,
                          booking_id_t const& id) {
    auto it = std::upper_bound(v.cbegin(), v.cend(), id);
    v.insert(it, id);
  };
  for (auto const& b_id : pseudo_gsd_bookings_) {
    if (concreteness(bookings_[b_id].r_) == 0) {

      insert_sorted(concrete_bookings_, b_id);
    } else {
      insert_sorted(mcf_bookings_, b_id);
    }
  }
  pseudo_gsd_bookings_.resize(0);
  pseudo_gsd_seats_.resize(0);
  for (auto const& [idx, gsd_id] : utl::enumerate(gsd_bookings_)) {
    reservation res = seat_attributes.at(gsd_seats_[idx]).second;
    for (auto seg = small_station_id_t{0}; seg != number_segments_; ++seg) {
      if (bookings_[gsd_id].interval_.from_ <= seg &&
          bookings_[gsd_id].interval_.to_ > seg) {
        capacities_.at(std::make_pair(res, seg))++;
      }
    }
  }
}
}  // namespace seat
