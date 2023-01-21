#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "utl/enumerate.h"

#include "ortools/linear_solver/linear_solver.h"

#include "seat/booking.h"

namespace seat {

namespace gor = operations_research;

constexpr auto const solver_name = "SCIP";

struct interval_graph {
  using node_id_t = std::uint16_t;
  using seat_number_t = std::uint16_t;

  interval_graph(std::vector<reservation> s)
      : seat_properties_{std::move(s)},
        solver_{gor::MPSolver::CreateSolver(solver_name)}
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
  }

  bool solve() {
    constraints_.clear();
    helpers_.clear();
    node_colors_.resize(booking_reservation_.size());
    int M = 55;
    for (auto const& [b_idx, b] : utl::enumerate(booking_reservation_)) {
      node_colors_[b_idx]=solver_->MakeIntVar(
          0.0, std::numeric_limits<double>::infinity(), fmt::format("color{}", b_idx));}
    for (auto const& [b_idx, b] : utl::enumerate(booking_reservation_)) {
      std::vector<interval> match_intervals;

      int from = -1, to = -1;
      for (auto const& [seat_idx, sr] : utl::enumerate(seat_properties_)) {
        auto const match = matches(b, sr);
        if (match && from == -1) {
          from = seat_idx;
          to = -1;
        }
        if (!match && from != -1) {
          to = seat_idx;
          match_intervals.emplace_back(interval{static_cast<uint8_t>(from), static_cast<uint8_t>(to)});
          from = -1;
        }
      }
      if(from!=-1){
        match_intervals.emplace_back(interval{static_cast<uint8_t>(from), static_cast<uint8_t>(seat_properties_.size()-1)});
      }

      auto const color = node_colors_[b_idx];

      auto j = 0U;
      auto const helpers_c = solver_->MakeRowConstraint(1,std::numeric_limits<double>::infinity(),"");
      constraints_.emplace_back(helpers_c);
      for (auto const& [from, to] : match_intervals) {
        auto const helper = solver_->MakeBoolVar(fmt::format("{}_h{}", b_idx, j++));
        auto const c1 = solver_->MakeRowConstraint(from-M,std::numeric_limits<double>::infinity(),"");
        c1->SetCoefficient(color,1);
        c1->SetCoefficient(helper,-M);
        auto const c2 = solver_->MakeRowConstraint(-to-M,std::numeric_limits<double>::infinity(),"");
        c2->SetCoefficient(color,-1);
        c2->SetCoefficient(helper,-M);
        helpers_c->SetCoefficient(helper,1);
        constraints_.emplace_back(c1);
        constraints_.emplace_back(c2);
      }
    }

    for(auto const& [c_id,c]:utl::enumerate(node_colors_)){
      if(neighbors_[c_id].empty()){
        break;
      }
      auto const c1 = solver_->MakeRowConstraint(0.5,std::numeric_limits<double>::infinity(),"");
      auto const c2 = solver_->MakeRowConstraint(-std::numeric_limits<double>::infinity(),M-0.5,"");
      for(auto const& n:neighbors_[c_id]){
        auto const neighbour = node_colors_[n];
        auto const helper_bool = solver_->MakeBoolVar(fmt::format("{}_h_{}", c_id,n));
        helpers_.emplace_back(helper_bool);
        c1->SetCoefficient(c,1);
        c1->SetCoefficient(neighbour,-1);
        c1->SetCoefficient(helper_bool,M);

        c2->SetCoefficient(c,1);
        c2->SetCoefficient(neighbour,-1);
        c2->SetCoefficient(helper_bool,M);
        constraints_.emplace_back(c1);
        constraints_.emplace_back(c2);
      }
    }
    auto bb=solver_->Solve();
    //std::cout<<"\n";
    for(auto const& [id,i]:utl::enumerate(node_colors_)){
      //std::cout<<"color "<<id<<": "<<i->solution_value()<<"\n";
    }
    for(auto const& [id,i]:utl::enumerate(helpers_)){
      //std::cout<<i->name()<<": "<<i->solution_value()<<"\n";
    }
    auto str = std::string{};
    solver_->ExportModelAsLpFormat(false,&str);
    std::cout<<str;
    return bb;
  }

  std::vector<gor::MPConstraint*> constraints_;
  std::vector<reservation> seat_properties_;
  std::vector<reservation> booking_reservation_;
  std::vector<interval> interval_;
  std::vector<std::vector<node_id_t>> neighbors_;

  std::unique_ptr<operations_research::MPSolver> solver_;
  std::vector<operations_research::MPVariable*> node_colors_;
  std::vector<operations_research::MPVariable*> helpers_;
};

}  // namespace seat
