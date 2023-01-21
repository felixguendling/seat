#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <random>

#include "utl/enumerate.h"

#include "seat/booking.h"

namespace seat {

  struct interval_graph {
    using node_id_t = std::uint16_t;
    using seat_number_t = std::uint16_t;

    interval_graph(std::vector<reservation> s)
        : seat_properties_{std::move(s)}
    {}

    void reset() {}

    void add_booking(booking const& b, bool replay = false) {
      std::vector<node_id_t> overlap;
      for (auto const& [node_id, interval] : utl::enumerate(interval_)) {
        if (b.interval_.overlaps(interval)) {
          overlap.emplace_back(node_id);
        }
      }
      neighbors_.emplace_back(overlap);
      for(auto const& o:overlap){
        neighbors_[o].emplace_back(neighbors_.size()-1);
      }
      booking_reservation_.emplace_back(b.r_);
      interval_.emplace_back(b.interval_);
      valid_colors_.emplace_back(find_valid_colors(b.r_));
    }

    std::vector<interval> find_valid_colors(reservation r) {
        std::vector<interval> match_intervals;

        int from = -1, to = -1;
        for (auto const& [seat_idx, sr] : utl::enumerate(seat_properties_)) {
          auto const match = matches(r, sr);
          if (match && from == -1) {
            from = seat_idx;
            to = -1;
          }
          if (!match && from != -1) {
            to = seat_idx;
            match_intervals.emplace_back(
                interval{static_cast<uint8_t>(from), static_cast<uint8_t>(to)});
            from = -1;
          }
        }
        if (from != -1) {
          match_intervals.emplace_back(
              interval{static_cast<uint8_t>(from),
                       static_cast<uint8_t>(seat_properties_.size() - 1)});
        }
        return match_intervals;
    }

    void greedy(){
      permutation_.resize(booking_reservation_.size());
      for(auto i=0;i!=permutation_.size();++i){
        permutation_[i] = i;
      }
      std::sort(begin(permutation_),end(permutation_),[&](int const a,int const b){
        return std::tuple{interval_[a].size(),concreteness(booking_reservation_[a])}>std::tuple{interval_[b].size(),concreteness(booking_reservation_[b])};
      });
      for(auto booking_idx:permutation_) {
        auto const r = booking_reservation_[booking_idx];
        auto const i = interval_[booking_idx];
        auto max = std::numeric_limits<int>::min();
        auto max_r = 0;
        for (auto j = 0; j != free_.size(); ++j) {
          if (!matches(r, j)) {
            continue;
          }
          auto const& free = free_[j];
          auto min = std::numeric_limits<int>::max();
          for (auto k = i.from_; k != i.to_; ++k) {
            if (free[k] < min) {
              min = free[k];
            }
          }
          if (min > max) {
            max = min;
            max_r = j;
          }
        }
        for (auto const [s_idx, sr] : utl::enumerate(seat_properties_)) {
          if (to_int(sr) != max_r) {
            continue;
          }
          for(auto const [chosen_idx,chosen]:utl::enumerate(chosen_seats)){
            if(chosen!=s_idx){
              continue;
            }
            if(interval_[chosen_idx].overlaps(i)){
              continue;
            }
            chosen_seats[booking_idx] = chosen;
            for(auto l=i.from_;l!=i.to_;++l){
              free_[max_r][l]--;
            }
            break;
          }
        }
        if(-1==chosen_seats[booking_idx]){
          auto min = std::numeric_limits<int>::max();
          auto min_idx = -1;
          for (auto const [s_idx, sr] : utl::enumerate(seat_properties_)) {
            if (to_int(sr) != max_r) {
              continue;
            }
            auto counter = std::count_if(neighbors_[booking_idx].begin(), neighbors_[booking_idx].end(), [&](node_id_t const neighbour_booking_idx){
              return s_idx==chosen_seats[neighbour_booking_idx];
            });
            if(counter<min){
              min = counter;
              min_idx = s_idx;
            }
          }
          chosen_seats[booking_idx]=min_idx;
        }
      }
    }

  std::vector<int> chosen_seats;
  std::vector<reservation> seat_properties_;
  std::vector<reservation> booking_reservation_;
  std::vector<interval> interval_;
  std::vector<std::vector<node_id_t>> neighbors_;
  std::vector<std::vector<interval>> valid_colors_;
  std::vector<int> permutation_;
  // außen reservation-kombination, innen sections
  std::vector<std::vector<int>> free_;

};
}