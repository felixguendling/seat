#include "seat/flow_graph.h"

#include <algorithm>
#include <limits>
#include <ostream>

#include "utl/enumerate.h"
#include "utl/get_or_create.h"

#include "seat/heuristic.h"
#include "seat/solver.h"

namespace seat {

std::ostream& operator<<(std::ostream& out, flow_graph::node const& n) {
  return out << static_cast<char>('A' + n.station_) << "_" << n.r_
             << (n.type_ == node_type::kSource ? "S"
                 : n.type_ == node_type::kSink ? "T"
                                               : "");
}

flow_graph::flow_graph(
    std::map<reservation, std::uint32_t> const& number_of_seats,
    unsigned const number_of_segments)
    : solver_{std::make_unique<solver>(*this)} {
  for (auto const& [r, c] : number_of_seats) {
    std::optional<node_id_t> pred;
    for (auto i = 0U; i != number_of_segments + 1; ++i) {
      auto const node_id = static_cast<node_id_t>(nodes_.size());
      nodes_.emplace_back(
          node{.r_ = r, .e_ = 0, .station_ = i, .type_ = node_type::kSegment});
      out_edges_.resize(nodes_.size());
      in_edges_.resize(nodes_.size() + 1);
      station_nodes_.resize(
          std::max(station_nodes_.size(), static_cast<std::size_t>(i + 1)));
      station_nodes_[i].emplace(r, node_id);

      if (pred.has_value()) {
        out_edges_[*pred].emplace_back(
            edge{.target_ = node_id, .capacity_ = c});
        in_edges_[node_id].emplace_back(edge{.target_ = *pred, .capacity_ = c});
      }

      pred = std::make_optional(node_id);
    }
  }
  solver_->add_fccs();
}

flow_graph::~flow_graph() = default;

auto flow_graph::get_create_node_fn(station_id_t const s,
                                    reservation const booking_r,
                                    bool const is_source) {
  return [&, booking_r, s, is_source]() {
    auto const node_id = static_cast<node_id_t>(nodes_.size());
    nodes_.emplace_back(
        node{.r_ = booking_r,
             .e_ = 0U,
             .station_ = s,
             .type_ = is_source ? node_type::kSource : node_type::kSink});
    out_edges_.resize(nodes_.size());
    in_edges_.resize(nodes_.size());
    solver_->set_excess(node_id, 0U);

    for (auto const& [node_r, n] : station_nodes_[s]) {
      if (matches(booking_r, node_r)) {
        auto const from = is_source ? node_id : n;
        auto const to = is_source ? n : node_id;
        out_edges_[from].emplace_back(
            edge{.target_ = to,
                 .capacity_ = std::numeric_limits<capacity_t>::max()});
        in_edges_[to].emplace_back(
            edge{.target_ = from,
                 .capacity_ = std::numeric_limits<capacity_t>::max()});
        solver_->add_flow(from, to);
      }
    }

    return node_id;
  };
}

capacity_t flow_graph::get_capacity(node_id_t const from, node_id_t const to) {
  if (auto const e = get_edge(from, to); e.has_value()) {
    return (*e)->capacity_;
  } else {
    return std::numeric_limits<capacity_t>::max();
  }
}

std::optional<flow_graph::edge*> flow_graph::get_edge(node_id_t const from,
                                                      node_id_t const to) {
  auto const it = std::find_if(begin(out_edges_[from]), end(out_edges_[from]),
                               [&](edge const& e) { return e.target_ == to; });
  if (it == end(out_edges_[from])) {
    return std::nullopt;
  } else {
    return std::make_optional(&(*it));
  }
}

void flow_graph::add_booking(booking const& b) {
  auto const source_node_id =
      utl::get_or_create(source_nodes_, std::pair{b.from_, b.r_},
                         get_create_node_fn(b.from_, b.r_, true));
  auto const sink_node_id =
      utl::get_or_create(sink_nodes_, std::pair{b.to_, b.r_},
                         get_create_node_fn(b.to_, b.r_, false));

  solver_->set_excess(source_node_id, ++nodes_[source_node_id].e_);
  solver_->set_excess(sink_node_id, --nodes_[sink_node_id].e_);
}

void flow_graph::to_graphviz(std::ostream& out, bool const print_in_edges) {
  out << "digraph R {\n";
  out << "  node [shape=record]\n";
  out << "  rankdir = LR;\n";

  for (auto const& [station_id, r_n] : utl::enumerate(station_nodes_)) {
    out << "  { rank=same ";
    for (auto const& [r, n] : r_n) {
      out << "  " << nodes_[n] << " ";
    }
    out << "}\n";
  }

  out << "  { rank=source ";
  for (auto const& [s_r, node_id] : source_nodes_) {
    auto const& [station_id, r] = s_r;
    out << nodes_[node_id] << " ";
  }
  out << "}\n";

  out << "  { rank=sink ";
  for (auto const& [s_r, node_id] : sink_nodes_) {
    auto const& [station_id, r] = s_r;
    out << nodes_[node_id] << " ";
  }
  out << "}\n";

  for (auto const& [station_id, r_n] : utl::enumerate(station_nodes_)) {
    for (auto const& [r, n] : r_n) {
      out << "  " << nodes_[n] << " [label = \""
          << static_cast<char>('A' + station_id) << " " << r << "\"];\n";
    }
  }

  for (auto const& [s_r, node_id] : source_nodes_) {
    auto const& [station_id, r] = s_r;
    out << "  " << nodes_[node_id] << " [label = \""
        << static_cast<char>('A' + station_id) << " " << r << " "
        << nodes_[node_id].e_ << "\"  color=\"green\"];\n";
  }

  for (auto const& [s_r, node_id] : sink_nodes_) {
    auto const& [station_id, r] = s_r;
    out << "  " << nodes_[node_id] << " [label = \""
        << static_cast<char>('A' + station_id) << " " << r << " "
        << nodes_[node_id].e_ << "\" color=\"red\"];\n";
  }

  for (auto const& [node_id, edges] : utl::enumerate(out_edges_)) {
    auto const& from = nodes_[node_id];
    for (auto const& e : edges) {
      auto const& to = nodes_[e.target_];
      out << "  " << from << " -> " << to;

      out << " [label=\"";
      out << e.flow_;
      if (e.capacity_ != std::numeric_limits<capacity_t>::max()) {
        out << " / " << e.capacity_;
      }
      out << "\"];\n";
    }
  }

  if (print_in_edges) {
    for (auto const& [node_id, edges] : utl::enumerate(in_edges_)) {
      auto const& from = nodes_[node_id];
      for (auto const& e : edges) {
        auto const& to = nodes_[e.target_];
        out << "  " << from << " -> " << to << " [";
        if (e.capacity_ != std::numeric_limits<capacity_t>::max()) {
          out << "label=\"" << e.capacity_ << "\" ";
        }
        out << "style=dashed];\n";
      }
    }
  }

  out << "}\n\n\n";
}

bool flow_graph::solve() {
  auto const feasible = solver_->solve();
  if (feasible) {
    solver_->write_flows();
  }
  return feasible;
}

std::string flow_graph::lp_str() const { return solver_->lp_str(); }

}  // namespace seat
