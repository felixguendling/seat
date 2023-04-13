#include "seat/verify_booking_assignment.h"

#include "utl/enumerate.h"

namespace seat {
verify::verify(std::vector<booking> const& bookings,
               std::vector<booking_id_t> const& mcf_ids,
               std::vector<booking_id_t> const& gsd_ids,
               std::vector<booking_id_t> const& pseudo_ids,
               std::vector<seat_id_t> const& mcf_seats,
               std::vector<seat_id_t> const& gsd_seats,
               std::vector<seat_id_t> const& pseudo_seats,
               seat_id_t const last_seat)
    : bookings_{bookings},
      mcf_ids_(mcf_ids),
      gsd_ids_(gsd_ids),
      pseudo_ids_(pseudo_ids),
      mcf_seats_(mcf_seats),
      gsd_seats_(gsd_ids),
      pseudo_seats_(pseudo_seats) {
  auto const find = [&](std::vector<booking_id_t> b_ids,
                        std::vector<seat_id_t> s_ids, seat_id_t s_id) {
    auto found_b = std::vector<booking_id_t>();
    for (auto const& [idx, bs_id] : utl::enumerate(s_ids)) {
      if (s_id != bs_id) {
        continue;
      }
      found_b.emplace_back(b_ids[idx]);
    }
    return found_b;
  };
  for (auto s_id = seat_id_t{0}; s_id != last_seat; ++s_id) {
    auto v = find(mcf_ids, mcf_seats, s_id);
    auto v2 = find(gsd_ids, gsd_seats, s_id);
    auto v3 = find(pseudo_ids, pseudo_seats, s_id);
    if (v.size() + v2.size() + v3.size() > 1) {
      std::cout << "\n";
      auto const cl = [&](std::vector<booking_id_t> vv) {
        for (auto const& b_id1 : vv) {
          for (auto const& b_id2 : vv) {
            if (b_id1 == b_id2) {
              continue;
            }
            auto i1 = bookings_[b_id1].interval_;
            auto i2 = bookings_[b_id2].interval_;
            if ((i1.from_ <= i2.from_ && i1.to_ >= i2.to_) ||
                (i1.from_ >= i2.from_ && i1.to_ <= i2.to_)) {
              std::cout << +b_id1 << ", " << +b_id2 << "  \n";
            }
          }
        }
      };
      cl(v);
      cl(v2);
      cl(v3);
      auto p = [&](std::vector<booking_id_t> b_ids) {
        for (auto const& id : b_ids) {
          std::cout << id << ", ";
        }
      };
      p(v);
      p(v2);
      p(v3);
      std::cout << "seat: " << s_id;
    }
  }
};
}  // namespace seat