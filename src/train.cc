#include "seat/train.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

#include "seat/booking.h"

namespace seat {
seat_cluster::seat_cluster(const wish table, const wish big,
                           const uint8_t seats_per_row, seat_id_t& first_seat)
    : table_{table}, big_{big} {
  init(table, big, seats_per_row, first_seat);
}

seat_cluster::seat_cluster(const wish table, const wish big,
                           seat_id_t& first_seat)
    : table_{table}, big_{big} {
  if (big == wish::kYes) {
    init(table, big, 3U, first_seat);
  } else {
    init(table, big, 4U, first_seat);
  }
}

void seat_cluster::init(const wish table, const wish big,
                        const uint8_t seats_per_row, seat_id_t& first_seat) {
  if (seats_per_row < 2 || table == wish::kAny || big == wish::kAny) {
    abort();
  }
  if (table == wish::kYes || big == wish::kYes) {
    if (table == wish::kYes && big == wish::kYes) {
      abort();
    }
    insert_row(first_seat, seats_per_row);
    insert_row(first_seat, seats_per_row);
  } else {
    insert_row(first_seat, seats_per_row);
  }
  last_id_ = first_seat;
}

void seat_cluster::insert_row(seat_id_t& first_seat,
                              uint8_t const seats_per_row) {
  auto vw = std::vector<seat_id_t>{};
  auto vc = std::vector<seat_id_t>{};
  vw.emplace_back(first_seat++);
  for (auto i = seat_id_t{first_seat}; i != first_seat + seats_per_row - 2;
       ++i) {
    vc.emplace_back(i);
  }
  first_seat += seats_per_row - 2;
  vw.emplace_back(first_seat++);
  window_seats_.emplace_back(vw);
  corridor_seats_.emplace_back(vc);
}

wagon::wagon(wish const silent, seat_id_t& last_id)
    : last_id_(last_id), silent_(silent) {
  if (silent == wish::kAny) {
    abort();
  }
}

void wagon::increment_seats_by_reservation_map(
    std::map<reservation, uint32_t>& seats_by_res) {
  auto res = reservation{};
  res[1] = silent_;
  for (auto const& row : seat_clusters_) {
    res[3] = row.big_;
    res[2] = row.table_;
    res[0] = wish::kYes;
    for (auto const& v : row.window_seats_) {
      for (auto const& s_id : v) {
        seats_by_res[res]++;
      }
    }
    res[0] = wish::kNo;
    for (auto const& v : row.corridor_seats_) {
      for (auto const& s_id : v) {
        seats_by_res[res]++;
      }
    }
  }
}

wish roll(double const& chance) {
  double r = (rand() % 10000) / double{10000};
  return chance > r ? wish::kYes : wish::kNo;
}

int wagon::roll_wagon_size(int const& max_wagon_size,
                           int const& min_wagon_size) {
  if (max_wagon_size == min_wagon_size) {
    size_ = min_wagon_size;
    return min_wagon_size;
  }
  auto r = rand() % (max_wagon_size - min_wagon_size) + min_wagon_size;
  size_ = r;
  return r;
}

void wagon::generate_random_wagon(int& min_num_of_seats,
                                  double const& table_chance,
                                  double const& big_chance,
                                  int const& max_wagon_size,
                                  int const& min_wagon_size) {
  auto size = roll_wagon_size(max_wagon_size, min_wagon_size);
  while (size > 0) {
    auto table = roll(table_chance);
    wish big;
    if (table == wish::kYes) {
      big = wish::kNo;
    } else {
      big = roll(big_chance);
    }
    min_num_of_seats += last_id_;
    size += last_id_;
    auto sc = seat_cluster(table, big, last_id_);
    seat_clusters_.emplace_back(sc);
    min_num_of_seats -= last_id_;
    size -= last_id_;
  }
}

void train::generate_random_train(int& min_num_of_seats,
                                  double const& silent_chance,
                                  double const& table_chance,
                                  double const& big_chance,
                                  int const& max_wagon_size,
                                  int const& min_wagon_size) {
  while (min_num_of_seats > 0) {
    auto silent = roll(silent_chance);
    auto w = wagon(silent, last_seat_id_);
    w.generate_random_wagon(min_num_of_seats, table_chance, big_chance,
                            max_wagon_size, min_wagon_size);
    last_seat_id_ = w.last_id_;
    train_.emplace(last_wagon_id_++, w);
  }
}

std::vector<std::vector<seat_id_t>> wagon::get_columns() const {
  auto columns = std::vector<std::vector<seat_id_t>>{};
  auto max =
      std::max_element(begin(seat_clusters_), end(seat_clusters_),
                       [](seat_cluster const& sc1, seat_cluster const& sc2) {
                         return sc1.corridor_seats_[0].size() <
                                sc2.corridor_seats_[0].size();
                       })
          ->corridor_seats_[0]
          .size() +
      2;
  auto rows = 0U;
  for (auto cluster = 0; cluster != seat_clusters_.size(); ++cluster) {
    auto const sc = seat_clusters_[cluster];
    for (auto row = 0; row != sc.window_seats_.size(); ++row) {
      rows += row;
      auto const window_seats = sc.window_seats_[row];
      auto const corridor_seats = sc.corridor_seats_[row];
      for (auto col = 0; col != max; ++col) {
        if (columns.size() == col) {
          columns.emplace_back(std::vector<seat_id_t>{});
        }
        uint32_t seat_id =
            ((col == 0) ? window_seats[0]
                        : (col == (corridor_seats.size() + 1)
                               ? window_seats[1]
                               : ((col > corridor_seats.size() + 1)
                                      ? std::numeric_limits<uint32_t>::max()
                                      : corridor_seats[col - 1])));
        columns[col].emplace_back(seat_id);
      }
    }
  }
  return columns;
}

void wagon::print(
    std::map<seat_id_t, std::pair<booking_id_t, booking>> const&
        seat_assignment =
            std::map<seat_id_t, std::pair<booking_id_t, booking>>{}) const {
  std::vector<std::vector<seat_id_t>> cols = get_columns();
  if (seat_assignment.empty()) {
    for (auto i = 0; i != cols.size(); ++i) {
      auto col = cols[i];
      for (auto row = 0; row != col.size(); ++row) {
        auto val = col[row];
        if (val == std::numeric_limits<uint32_t>::max()) {
          std::cout << "____|";
          continue;
        }
        auto print = val;
        std::cout << print << (print < 1000 ? " " : "")
                  << (print < 100 ? " " : "") << (print < 10 ? " " : "") << "|";
      }
      std::cout << "\n";
    }
  } else {
    for (auto i = 0; i != cols.size(); ++i) {
      auto col = cols[i];
      for (auto row = 0; row != col.size(); ++row) {
        auto val = col[row];
        if (val == std::numeric_limits<uint32_t>::max()) {
          std::cout << "____|";
          continue;
        }
        if (seat_assignment.find(val) == seat_assignment.end()) {
          std::cout << "xxxx|";
          continue;
        }
        auto print = seat_assignment.at(val).first;
        std::cout << print << (print < 1000 ? " " : "")
                  << (print < 100 ? " " : "") << (print < 10 ? " " : "") << "|";
      }
      std::cout << "\n";
    }
  }
}

void train::print() const {
  auto assignment = cista::raw::vector_map<
      station_id_t, std::map<seat_id_t, std::pair<booking_id_t, booking>>>{};
  print(assignment);
}

void train::print(
    cista::raw::vector_map<
        station_id_t,
        std::map<seat_id_t, std::pair<booking_id_t, booking>>> const&
        assignment = cista::raw::vector_map<
            station_id_t,
            std::map<seat_id_t, std::pair<booking_id_t, booking>>>{}) const {
  std::cout << "_____________________printing train_______________________\n";
  for (auto station = station_id_t{0}; station != assignment.size();
       ++station) {
    std::cout << "___________________station: " << station
              << "_______________________\n";
    for (auto const& [id, w] : train_) {
      std::cout << "wagon " << +id << ", (size: " << w.size_
                << (w.silent_ == wish::kYes ? " silent" : " handy") << "):\n";
      w.print(assignment[station]);
    }
  }
  if (assignment.empty()) {
    for (auto const& [id, w] : train_) {
      std::cout << "wagon " << +id << ", (size: " << w.size_ << "):\n";
      w.print();
    }
  }
}

void seat_cluster::get_seat_attributes(
    std::map<seat_id_t, std::pair<wagon_id_t, reservation>>& attributes,
    wagon_id_t const& w_id, wish const silent) const {
  auto r = reservation{};
  r[1] = silent;
  r[2] = table_;
  r[3] = big_;
  r[0] = wish::kYes;
  for (auto const& v : window_seats_) {
    for (auto const& s_id : v) {
      auto res = r;
      attributes[s_id] = std::make_pair(w_id, res);
    }
  }
  r[0] = wish::kNo;
  for (auto const& v : corridor_seats_) {
    for (auto const& s_id : v) {
      auto res = r;
      attributes[s_id] = std::make_pair(w_id, res);
    }
  }
}

void wagon::get_seat_attributes(
    std::map<seat_id_t, std::pair<wagon_id_t, reservation>>& attributes,
    wagon_id_t const& w_id) const {
  for (auto const& sc : seat_clusters_) {
    sc.get_seat_attributes(attributes, w_id, silent_);
  }
}

std::map<seat_id_t, std::pair<wagon_id_t, reservation>>
train::get_seat_attributes() const {
  std::map<seat_id_t, std::pair<wagon_id_t, reservation>> ret;
  for (auto const& [id, w] : train_) {
    w.get_seat_attributes(ret, id);
  }
  return ret;
}

void seat_cluster::get_number_of_seats(
    std::map<reservation, std::uint32_t>& seats, wish const silent) const {
  auto r = reservation();
  r[1] = silent;
  r[2] = table_;
  r[3] = big_;
  r[0] = wish::kYes;
  for (auto const& v : window_seats_) {
    for (auto const& s : v) {
      seats[r]++;
    }
  }
  r[0] = wish::kNo;
  for (auto const& v : corridor_seats_) {
    for (auto const& s : v) {
      seats[r]++;
    }
  }
}

void wagon::get_number_of_seats(
    std::map<reservation, std::uint32_t>& seats) const {
  for (auto const& sc : seat_clusters_) {
    sc.get_number_of_seats(seats, silent_);
  }
}

reservation train::get_possible_reservations() const {
  auto r = reservation{};
  auto first_wagon = train_.at(wagon_id_t{0});
  r[1] = first_wagon.silent_;
  auto first_sc = first_wagon.seat_clusters_[0];
  r[2] = first_sc.table_;
  r[3] = first_sc.big_;
  r[0] = wish::kAny;
  for (auto const& [id, w] : train_) {
    if (r[1] != w.silent_) {
      r[1] = wish::kAny;
    }
    for (auto const& sc : w.seat_clusters_) {
      if (r[2] != sc.table_) {
        r[2] = wish::kAny;
      }
      if (r[3] != sc.big_) {
        r[3] = wish::kAny;
      }
    }
  }
  return r;
}

std::map<reservation, std::uint32_t> train::get_number_of_seats() const {
  std::map<reservation, std::uint32_t> ret;
  auto r = get_possible_reservations();
  auto reservations = get_concrete_reservations(r);
  for (auto const& res : reservations) {
    ret.emplace(res, 0);
  }
  for (auto const& [id, w] : train_) {
    w.get_number_of_seats(ret);
  }
  return ret;
}
}  // namespace seat