#pragma once

#include <map>

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"
#include "seat/wish.h"

namespace seat {

struct seat_cluster {
  explicit seat_cluster(wish const, wish const, uint8_t const, seat_id_t&);
  explicit seat_cluster(wish const, wish const, seat_id_t&);
  void init(const wish, const wish, const uint8_t, seat_id_t&);
  void get_number_of_seats(std::map<reservation, std::uint32_t>& seats,
                           wish const) const;
  void insert_row(seat_id_t&, uint8_t const);
  void get_seat_attributes(
      std::map<seat_id_t, std::pair<wagon_id_t, reservation>>&,
      const wagon_id_t&, const wish) const;

  wish table_;
  wish big_;
  std::vector<std::vector<seat_id_t>> window_seats_;
  std::vector<std::vector<seat_id_t>> corridor_seats_;
  seat_id_t last_id_;
};

struct wagon {
  explicit wagon(wish const, seat_id_t&);

  std::vector<std::vector<seat_id_t>> get_columns() const;
  void increment_seats_by_reservation_map(std::map<reservation, uint32_t>&);
  void generate_random_wagon(int&, double const&, double const&, int const&,
                             int const&);
  void print() const;
  void print(small_station_id_t const, std::vector<booking_id_t> const&,
             std::vector<booking_id_t> const&, std::vector<booking_id_t> const&,
             std::vector<seat_id_t> const&, std::vector<seat_id_t> const&,
             std::vector<seat_id_t> const&, std::vector<booking> const&) const;
  int roll_wagon_size(int const&, int const&);
  void get_number_of_seats(std::map<reservation, std::uint32_t>& seats) const;
  void get_seat_attributes(
      std::map<seat_id_t, std::pair<wagon_id_t, reservation>>&,
      wagon_id_t const&) const;

  wish silent_;
  std::vector<seat_cluster> seat_clusters_;
  seat_id_t last_id_;
  int size_;
};

struct train {
  reservation get_possible_reservations() const;
  void generate_random_train(int&, double const&, double const&, double const&,
                             int const& max_wagon_size,
                             int const& min_wagon_size);

  void print(small_station_id_t const, std::vector<booking_id_t> const&,
             std::vector<booking_id_t> const&, std::vector<booking_id_t> const&,
             std::vector<seat_id_t> const&, std::vector<seat_id_t> const&,
             std::vector<seat_id_t> const&, std::vector<booking> const&) const;
  void print() const;
  std::map<seat_id_t, std::pair<wagon_id_t, reservation>> get_seat_attributes()
      const;
  std::map<reservation, std::uint32_t> get_number_of_seats() const;
  std::vector<seat_id_t> get_available_seats(
      std::vector<reservation> const& available_res,
      std::map<seat_id_t, std::vector<booking>> const& occupied);
  template <typename F>
  void for_each_seat(F&&);
  void get_reservation(seat_id_t const&, reservation&);
  std::vector<seat_id_t> get_seats(reservation const&);
  std::map<std::pair<wagon_id_t, reservation>, uint32_t>
  get_wagon_res_capacities(std::vector<seat_id_t> const&);
  wagon_id_t seat_id_to_wagon_id(seat_id_t const&);

  seat_id_t last_seat_id_;
  wagon_id_t last_wagon_id_;
  std::map<wagon_id_t, wagon> train_;
};
}  // namespace seat
