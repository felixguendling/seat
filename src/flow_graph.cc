#include "seat/flow_graph.h"

#include <limits>
#include <ostream>

#include "utl/enumerate.h"
#include "utl/get_or_create.h"

#include "seat/solver.h"

namespace seat {

std::ostream& operator<<(std::ostream& out, flow_graph::node const& n) {
  return out << static_cast<char>('A' + n.station_) << "_" << n.r_
             << (n.type_ == node_type::kSource ? "S"
                 : n.type_ == node_type::kSink ? "TS"
                                               : "");
}

flow_graph::flow_graph(flow_graph const& copy_me)
    : nodes_(copy_me.nodes_),
      out_edges_(copy_me.out_edges_),
      in_edges_(copy_me.in_edges_),
      station_nodes_(copy_me.station_nodes_),
      source_nodes_(copy_me.source_nodes_),
      sink_nodes_(copy_me.sink_nodes_),
      solver_(std::make_unique<solver>(*this)) {}

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
void flow_graph::add_booking(const seat::booking& b) { add_booking(b, true); }

void flow_graph::add_booking(booking const& b, bool const insert) {
  auto const source_node_id =
      utl::get_or_create(source_nodes_, std::pair{b.interval_.from_, b.r_},
                         get_create_node_fn(b.interval_.from_, b.r_, true));
  auto const sink_node_id =
      utl::get_or_create(sink_nodes_, std::pair{b.interval_.to_, b.r_},
                         get_create_node_fn(b.interval_.to_, b.r_, false));

  solver_->set_excess(source_node_id, ++nodes_[source_node_id].e_);
  solver_->set_excess(sink_node_id, --nodes_[sink_node_id].e_);
  if (insert) {
    mcf_bookings_.emplace_back(b);
  }
}

void flow_graph::to_graphviz(std::ostream& out,
                             bool const print_in_edges) const {
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
void flow_graph::remove_booking(const seat::booking& b) {
  remove_booking(b, true);
}

void flow_graph::remove_booking(booking const& b, bool remove) {
  auto const source_node_id =
      utl::get_or_create(source_nodes_, std::pair{b.interval_.from_, b.r_},
                         get_create_node_fn(b.interval_.from_, b.r_, true));
  auto const sink_node_id =
      utl::get_or_create(sink_nodes_, std::pair{b.interval_.to_, b.r_},
                         get_create_node_fn(b.interval_.to_, b.r_, false));

  solver_->set_excess(source_node_id, --nodes_[source_node_id].e_);
  solver_->set_excess(sink_node_id, ++nodes_[sink_node_id].e_);
  auto it = std::find(mcf_bookings_.begin(), mcf_bookings_.end(), b);
  if (remove) {
    mcf_bookings_.erase(it);
  }
}

std::string flow_graph::lp_str() const { return solver_->lp_str(); }

void flow_graph::print_sizes() const { solver_->print_sizes(); }

void flow_graph::print_name() const { std::cout << "flow graph solver"; }

std::map<reservation, bool> flow_graph::gsd_request(interval const& i) {
  std::map<reservation, bool> available_res;
  auto m = station_nodes_[0];
  for (auto const& [r, s] : m) {
    auto b = booking{};
    b.r_ = r;
    b.interval_ = i;
    add_booking(b, false);
    auto feasible = solve();
    available_res.insert(std::make_pair(r, feasible));
    remove_booking(b, false);
  }
  return available_res;
}

void flow_graph::add_gsd_booking(booking const& gsd_b, uint32_t const& seat) {
  gsd_bookings_.insert(std::make_pair(seat, gsd_b));
  add_booking(gsd_b, false);
  solve();
  auto removable_bookings =
      get_removable_bookings(gsd_b, get_reached_capacity_ids(gsd_b.r_));

  remove_booking(gsd_b, false);
}

std::vector<bool> flow_graph::get_removable_bookings(
    booking const& gsd_b,
    std::vector<std::pair<node_id_t, node_id_t>> const& capacity_reached) {
  auto removable_bookings = std::vector<bool>{};
  removable_bookings.resize(mcf_bookings_.size());
  std::fill(removable_bookings.begin(), removable_bookings.end(), true);
  for (auto const& [i, b] : utl::enumerate(mcf_bookings_)) {
    if (b.interval_.overlaps(gsd_b.interval_)) {
      removable_bookings[i] = false;
      continue;
    }
    // only consider bookings whose interval contains index for which the
    // capacity has been reached.
    auto found = false;
    for (auto const& [from, to] : capacity_reached) {
      if (from == b.interval_.from_ && to == b.interval_.to_) {
        found = true;
        break;
      }
    }
    if (!found) {
      removable_bookings[i] = false;
    }

    if (!matches(b.r_, gsd_b.r_)) {
      removable_bookings[i] = false;
    }
  }
  return removable_bookings;
}

std::vector<std::pair<node_id_t, node_id_t>>
flow_graph::get_reached_capacity_ids(reservation const& r) {
  auto capacity_reached = std::vector<std::pair<node_id_t, node_id_t>>{};
  for (auto station_id = 0; station_id != station_nodes_.size(); ++station_id) {
    for (auto const& [res, node_id] : station_nodes_[station_id]) {
      if (!matches(res, r)) {
        continue;
      }
      auto node = nodes_[node_id];
      if (node.type_ != node_type::kSegment) {
        continue;
      }
      auto outs = out_edges_[node_id];
      auto in = in_edges_[node_id];
      for (auto const& edge : out_edges_[node_id]) {
        auto const target = nodes_[edge.target_];
        if (target.type_ != node_type::kSegment) {
          continue;
        }
        // check (1) is capacity reached on this edge? (2) has the found pair
        // already been inserted in reverse order.
        if (edge.capacity_ ==
                solver_->get_occupied_seats(node_id, edge.target_) &&
            (std::find(begin(capacity_reached), end(capacity_reached),
                       std::make_pair(edge.target_, node_id)) ==
             end(capacity_reached))) {
          capacity_reached.emplace_back(std::make_pair(node_id, edge.target_));
        }
      }
      for (auto const& edge : in_edges_[node_id]) {
        auto const target = nodes_[edge.target_];
        if (target.type_ != node_type::kSegment) {
          continue;
        }
        // check (1) is capacity reached on this edge? (2) has the found pair
        // already been inserted in reverse order.
        if (edge.capacity_ ==
                solver_->get_occupied_seats(node_id, edge.target_) &&
            (std::find(begin(capacity_reached), end(capacity_reached),
                       std::make_pair(edge.target_, node_id)) ==
             end(capacity_reached))) {
          capacity_reached.emplace_back(std::make_pair(node_id, edge.target_));
        }
      }
    }
  }
  return capacity_reached;
}

void flow_graph::print() const { solver_->print(); }
cista::raw::vector_map<booking_id_t, booking> flow_graph::get_mcf_bookings() {
  return cista::raw::vector_map<booking_id_t, booking>{};
}
std::map<seat_id_t, std::vector<booking>> flow_graph::get_gsd_bookings() {
  return std::map<seat_id_t, std::vector<booking>>{};
}
}  // namespace seat
