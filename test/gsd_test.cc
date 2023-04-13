#include "gtest/gtest.h"

#include "seat/booking.h"
#include "seat/nogo_cache.h"
#include "seat/seat_assignment.h"
#include "seat/solver.h"
#include "seat/train.h"

using namespace seat;

TEST(gsd, works) {
  auto res = reservation{wish::kNo, wish::kNo, wish::kNo, wish::kNo};
  auto const b1 = booking{.r_ = {wish::kNo, wish::kNo, wish::kNo, wish::kNo},
                          .interval_ = {1, 3}};
  auto const b2 = booking{.r_ = {wish::kNo, wish::kNo, wish::kNo, wish::kNo},
                          .interval_ = {4, 10}};
  auto const b3 = booking{.r_ = {wish::kNo, wish::kNo, wish::kNo, wish::kNo},
                          .interval_ = {0, 10}};
  auto t = train();
  auto s_id = seat_id_t{0};
  auto const w = wish::kNo;
  auto sc = seat_cluster(w, w, s_id);
  wagon wag = wagon(w, s_id);
  wag.size_ = 4;
  wag.seat_clusters_.emplace_back(sc);
  t.train_.emplace(wagon_id_t{0}, wag);
  t.print();
  auto const& gsd_prob = 0.4;
  auto const number_of_segments = 11U;
  auto seats = t.get_number_of_seats();

  auto fpb_solver = solver{seats, number_of_segments};
  nogo_cache c;
  fpb_solver.add_booking(b1);
  fpb_solver.add_booking(b2);
  fpb_solver.add_booking(b3);

  auto inter = interval{3, 4};
  std::vector<reservation> valid = fpb_solver.gsd_request(inter);
  auto pos = std::find(valid.begin(), valid.end(), res) - valid.begin();
  std::vector<seat_id_t> valid_seats =
      t.get_available_seats(valid, fpb_solver.get_gsd_bookings());
  auto gsd = booking{};
  gsd.interval_ = inter;
  gsd.r_ = res;
  fpb_solver.add_gsd_booking(gsd, 1);
  auto tt = t.get_seat_attributes();
  auto mcf = fpb_solver.get_mcf_bookings();
  auto gsd2 = fpb_solver.get_gsd_bookings();
  auto seat_solver =
      solver_seat(number_of_segments, tt, mcf, gsd2, t.last_seat_id_);
  seat_solver.create_mcf_problem();
  auto feasible = seat_solver.solve();
  cista::raw::vector_map<station_id_t,
                         std::map<seat_id_t, std::pair<booking_id_t, booking>>>
      assignment = seat_solver.assign_seats();
  for (auto const& iq : assignment) {
    std::cout << "\n";
    for (auto const& [s_i, pair] : iq) {
      std::cout << s_i << " -> " << pair.second << "\n";
    }
  }
  t.print(assignment, fpb_solver.gsd_bookings_,
          fpb_solver.pseudo_gsd_bookings_);
}
