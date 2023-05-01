#include "seat/train.h"

#include <windows.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>

#include "seat/booking.h"
#include "utl/enumerate.h"
#include "utl/get_or_create.h"

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

void wagon::print() const {
  print(1, std::vector<booking_id_t>(), std::vector<booking_id_t>(),
        std::vector<booking_id_t>(), std::vector<seat_id_t>(),
        std::vector<seat_id_t>(), std::vector<seat_id_t>(),
        std::vector<booking>());
}

void print_b_ids_equal_length(booking_id_t const& b_id) {
  std::cout << b_id << (b_id < 1000 ? " " : "") << (b_id < 100 ? " " : "")
            << (b_id < 10 ? " " : "") << "|";
}

void wagon::print_seat_ids() const {
  std::vector<std::vector<seat_id_t>> cols = get_columns();
  for (auto i = 0; i != cols.size(); ++i) {
    auto col = cols[i];
    for (auto row = 0; row != col.size(); ++row) {
      auto val = col[row];
      if (val == std::numeric_limits<uint32_t>::max()) {
        std::cout << "____|";
        continue;
      }
      auto print = val;
      print_b_ids_equal_length(print);
    }
    std::cout << "\n";
  }
}

void wagon::print(small_station_id_t const segment,
                  std::vector<booking_id_t> const& mcf,
                  std::vector<booking_id_t> const& gsd,
                  std::vector<booking_id_t> const& pseudo,
                  std::vector<seat_id_t> const& mcf_seats,
                  std::vector<seat_id_t> const& gsd_seats,
                  std::vector<seat_id_t> const& pseudo_seats,
                  std::vector<booking> const& bookings) const {
  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

  seat_id_t s_id;
  auto find = [&](std::vector<booking_id_t> b_ids, std::vector<seat_id_t> s_ids,
                  int color) {
    for (auto const& [id, mcf_s_id] : utl::enumerate(s_ids)) {
      if (s_id != mcf_s_id) {
        continue;
      }
      if (bookings[b_ids[id]].interval_.from_ <= segment &&
          bookings[b_ids[id]].interval_.to_ > segment) {
        SetConsoleTextAttribute(hConsole, color);
        print_b_ids_equal_length(b_ids[id]);
        SetConsoleTextAttribute(hConsole, 15);  // default color (white)
        return true;
      }
    }
    return false;
  };
  for (auto row = 0; row != 4; ++row) {
    for (auto const& sc : seat_clusters_) {
      for (auto col = 0; col != sc.window_seats_.size(); ++col) {
        auto w_seats = sc.window_seats_[col];
        auto c_seats = sc.corridor_seats_[col];
        if (row == 3 && c_seats.size() == 1) {
          s_id = std::numeric_limits<seat_id_t>::max();
        } else if (row == 0) {
          s_id = w_seats[row];
        } else if (row == 1 + c_seats.size()) {
          s_id = w_seats[row - c_seats.size()];
        } else {
          s_id = c_seats[row - 1];
        }
        if (s_id == std::numeric_limits<seat_id_t>::max()) {
          std::cout << "____|";
          continue;
        }
        auto found = find(gsd, gsd_seats, 2);  // green->gsd bookings
        if (found) {
          continue;
        }
        found = find(pseudo, pseudo_seats, 3);  // blue-ish->pseudo_gsd bookings
        if (found) {
          continue;
        }
        found = find(mcf, mcf_seats, 12);
        if (!found) {
          if (std::find(gsd_seats.begin(), gsd_seats.end(), s_id) !=
              gsd_seats.end()) {
            SetConsoleTextAttribute(
                hConsole,
                2);  // gsd-color to indicate that seat has been gsd-blocked
          }
          std::cout << "xxxx|";
          SetConsoleTextAttribute(hConsole, 15);  // default color (white)
          continue;
        }
      }
    }
    std::cout << "\n";
  }
}

void wagon::print2(small_station_id_t const segment,
                   std::vector<booking> const& bookings,
                   std::pair<std::vector<booking_id_t>,
                             std::vector<seat_id_t>> const& normal_bookings,
                   std::vector<booking_id_t> const& gsd,
                   std::vector<seat_id_t> const& gsd_seats) const {
  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

  seat_id_t s_id;
  auto find = [&](std::vector<booking_id_t> b_ids, std::vector<seat_id_t> s_ids,
                  int color) {
    for (auto const& [id, mcf_s_id] : utl::enumerate(s_ids)) {
      if (s_id != mcf_s_id) {
        continue;
      }
      auto b = bookings[b_ids[id]];
      if (b.group_id_ != 0) {
        color = (b.group_id_ % 20) + 16;
      }
      if (b.interval_.from_ <= segment && b.interval_.to_ > segment) {
        SetConsoleTextAttribute(hConsole, color);
        print_b_ids_equal_length(b_ids[id]);
        SetConsoleTextAttribute(hConsole, 15);  // default color (white)
        return true;
      }
    }
    return false;
  };

  for (auto row = 0; row != 4; ++row) {
    for (auto const& sc : seat_clusters_) {
      for (auto col = 0; col != sc.window_seats_.size(); ++col) {
        auto w_seats = sc.window_seats_[col];
        auto c_seats = sc.corridor_seats_[col];
        if (row == 3 && c_seats.size() == 1) {
          s_id = std::numeric_limits<seat_id_t>::max();
        } else if (row == 0) {
          s_id = w_seats[row];
        } else if (row == 1 + c_seats.size()) {
          s_id = w_seats[row - c_seats.size()];
        } else {
          s_id = c_seats[row - 1];
        }
        if (s_id == std::numeric_limits<seat_id_t>::max()) {
          std::cout << "____|";
          continue;
        }
        auto found = find(gsd, gsd_seats, 2);  // green->gsd bookings
        if (found) {
          continue;
        }
        found = find(normal_bookings.first, normal_bookings.second, 12);
        if (!found) {
          if (std::find(gsd_seats.begin(), gsd_seats.end(), s_id) !=
              gsd_seats.end()) {
            SetConsoleTextAttribute(
                hConsole,
                2);  // gsd-color to indicate that seat has been gsd-blocked
          }
          std::cout << "xxxx|";
          SetConsoleTextAttribute(hConsole, 15);  // default color (white)
          continue;
        }
      }
    }
    std::cout << "\n";
  }
}

void train::print() const {
  auto assignment_dummy = std::vector<booking_id_t>();
  auto seats_dummy = std::vector<seat_id_t>();
  auto bookings_dummy = std::vector<booking>();
  print(1, assignment_dummy, assignment_dummy, assignment_dummy, seats_dummy,
        seats_dummy, seats_dummy, bookings_dummy);
}

void print_wagon_descr(int id, wagon w) {
  std::cout << "wagon " << +id << ", (size: " << w.size_
            << (w.silent_ == wish::kYes ? " silent" : " handy") << "):\n";
}

void train::print_seat_ids(output_options const& options) const {
  if (!options.do_output_) {
    return;
  }
  std::cout << "_____________________printing empty train_________________\n";
  for (auto const& [id, w] : train_) {
    print_wagon_descr(id, w);
    w.print_seat_ids();
  }
}

void train::print(small_station_id_t const last_station,
                  std::vector<booking_id_t> const& mcf,
                  std::vector<booking_id_t> const& gsd,
                  std::vector<booking_id_t> const& pseudo_gsd,
                  std::vector<seat_id_t> const& mcf_seats,
                  std::vector<seat_id_t> const& gsd_seats,
                  std::vector<seat_id_t> const& pseudo_seats,
                  std::vector<booking> const& bookings) const {
  std::cout << "_____________________printing train_______________________\n";
  for (auto station = station_id_t{0}; station != last_station; ++station) {
    std::cout << "___________________segment: " << station
              << "_______________________\n";
    for (auto const& [id, w] : train_) {
      print_wagon_descr(id, w);
      w.print(station, mcf, gsd, pseudo_gsd, mcf_seats, gsd_seats, pseudo_seats,
              bookings);
    }
  }
}

void train::print2(small_station_id_t const last_station,
                   std::vector<booking> const& bookings,
                   std::pair<std::vector<booking_id_t>,
                             std::vector<seat_id_t>> const& normal_bookings,
                   std::vector<booking_id_t> const& gsd,
                   std::vector<seat_id_t> const& gsd_seats) const {
  std::cout << "_____________________printing train_______________________\n";
  for (auto station = station_id_t{0}; station != last_station; ++station) {
    std::cout << "___________________segment: " << station
              << "_______________________\n";
    for (auto const& [id, w] : train_) {
      print_wagon_descr(id, w);
      w.print2(station, bookings, normal_bookings, gsd, gsd_seats);
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

std::vector<seat_id_t> train::get_available_seats(
    std::vector<reservation> const& available_res,
    std::map<seat_id_t, std::vector<booking>> const& occupied) {
  std::vector<seat_id_t> available_seats;
  auto collect_available_seats = [&](seat_id_t const& seat_id,
                                     reservation const& seat_res,
                                     wagon_id_t w_id) {
    if (std::find(available_res.begin(), available_res.end(), seat_res) ==
        available_res.end()) {
      return;
    }
    if (occupied.find(seat_id) != occupied.end()) {
      return;
    }
    available_seats.emplace_back(seat_id);
  };
  for_each_seat(collect_available_seats);
  return available_seats;
}

template <typename F>
void train::for_each_seat(F&& f) {
  reservation r;
  for (auto const& [w_id, w] : train_) {
    r[1] = w.silent_;
    for (auto const& sc : w.seat_clusters_) {
      r[2] = sc.table_;
      r[3] = sc.big_;
      r[0] = wish::kYes;
      for (auto const& row : sc.window_seats_) {
        for (auto const& s_id : row) {
          f(s_id, r, w_id);
        }
      }
      r[0] = wish::kNo;
      for (auto const& row : sc.corridor_seats_) {
        for (auto const& s_id : row) {
          f(s_id, r, w_id);
        }
      }
    }
  }
}

void train::get_reservation(seat_id_t const& s_id, reservation& r) {
  for_each_seat(
      [&](seat_id_t const& seat_id, reservation const& res, wagon_id_t w_id) {
        if (s_id == seat_id) {
          r = res;
          return;
        }
      });
}

std::vector<seat_id_t> train::get_seats(reservation const& r) {
  auto seats = std::vector<seat_id_t>();
  for_each_seat(
      [&](seat_id_t const& seat_id, reservation const& res, wagon_id_t w_id) {
        if (r == res) {
          seats.emplace_back(seat_id);
        }
      });
  return seats;
}

std::map<std::pair<wagon_id_t, reservation>, uint32_t>
train::get_wagon_res_capacities(std::vector<seat_id_t> const& gsd_seats) {
  auto wagon_res_cap = std::map<std::pair<wagon_id_t, reservation>, uint32_t>();
  for_each_seat([&](seat_id_t const& seat_id, reservation const& res,
                    wagon_id_t const& w_id) {
    if (find(gsd_seats.begin(), gsd_seats.end(), seat_id) != gsd_seats.end()) {
      return;
    }
    auto pair = std::make_pair(w_id, res);
    utl::get_or_create(wagon_res_cap, pair, []() { return 0; });
    wagon_res_cap[pair]++;
  });
  return wagon_res_cap;
}

wagon_id_t train::seat_id_to_wagon_id(seat_id_t const& s_id) {
  wagon_id_t w;
  for_each_seat(
      [&](seat_id_t const& seat_id, reservation const& res, wagon_id_t w_id) {
        if (s_id == seat_id) {
          w = w_id;
          return;
        }
      });
  return w;
}

}  // namespace seat