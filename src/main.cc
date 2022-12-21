
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "fmt/format.h"
#include "fmt/ostream.h"

#include "ortools/linear_solver/linear_solver.h"

#include "cista/containers/matrix.h"

#include "utl/enumerate.h"
#include "utl/get_or_create.h"
#include "utl/zip.h"

namespace gor = operations_research;

enum class properties { kWindow, kSilent, kTable, kBig, kNProperties };

constexpr auto const kNProperties =
    static_cast<std::underlying_type_t<properties>>(properties::kNProperties);

enum class wish : std::uint8_t { kNo, kYes, kAny };

using reservation = std::array<wish, kNProperties>;

std::ostream& operator<<(std::ostream& out, wish const w) {
  return out << (w == wish::kYes ? 'Y' : (w == wish::kNo ? 'N' : 'X'));
}

std::ostream& operator<<(std::ostream& out, reservation const& r) {
  for (auto const& w : r) {
    out << w;
  }
  return out;
}

bool matches(reservation const a, reservation const b) {
  for (auto const& [pa, pb] : utl::zip(a, b)) {
    if (pa != wish::kAny && pa != pb) {
      return false;
    }
  }
  return true;
}

bool is_concrete(reservation const a) {
  return std::all_of(begin(a), end(a),
                     [](wish const p) { return p != wish::kAny; });
}

using station_id_t = std::uint32_t;
using node_id_t = std::uint32_t;
using capacity_t = std::uint32_t;

struct node {
  friend std::ostream& operator<<(std::ostream& out, node const& n) {
    return out << static_cast<char>('A' + n.station_) << "_" << n.r_
               << (n.e_ > 0   ? "S"
                   : n.e_ < 0 ? "T"
                              : "");
  }
  reservation r_;
  std::int32_t e_;
  station_id_t station_;
};

struct booking {
  reservation r_;
  std::uint32_t from_, to_;
};

struct edge {
  node_id_t target_;
  capacity_t capacity_;
};

int main() {
  auto const number_of_segments = 2U;
  std::map<reservation, std::uint32_t> number_of_seats{
      {{wish::kNo, wish::kYes, wish::kNo, wish::kNo}, 2U},
      {{wish::kYes, wish::kYes, wish::kNo, wish::kNo}, 2U},
      {{wish::kNo, wish::kNo, wish::kNo, wish::kNo}, 2U},
      {{wish::kYes, wish::kNo, wish::kNo, wish::kNo}, 2U},
  };

  std::vector<node> nodes_;
  std::vector<std::vector<edge>> out_edges_;
  std::vector<std::vector<edge>> in_edges_;
  std::vector<std::map<reservation, node_id_t>> station_nodes;

  for (auto const& [r, c] : number_of_seats) {
    std::optional<node_id_t> pred;
    for (auto i = 0U; i != number_of_segments + 1; ++i) {
      auto const node_id = static_cast<node_id_t>(nodes_.size());
      nodes_.emplace_back(node{.r_ = r, .e_ = 0, .station_ = i});
      out_edges_.resize(nodes_.size());
      in_edges_.resize(nodes_.size() + 1);
      station_nodes.resize(
          std::max(station_nodes.size(), static_cast<std::size_t>(i + 1)));
      station_nodes[i].emplace(r, node_id);

      if (pred.has_value()) {
        out_edges_[*pred].emplace_back(
            edge{.target_ = node_id, .capacity_ = c});
        in_edges_[node_id].emplace_back(edge{.target_ = *pred, .capacity_ = c});
      }

      pred = std::make_optional(node_id);
    }
  }

  std::vector<booking> bookings{
      booking{.r_ = {wish::kAny, wish::kYes, wish::kAny, wish::kAny},
              .from_ = 0U,
              .to_ = 1U},
      booking{.r_ = {wish::kAny, wish::kYes, wish::kAny, wish::kAny},
              .from_ = 0U,
              .to_ = 2U},
      booking{.r_ = {wish::kYes, wish::kAny, wish::kAny, wish::kAny},
              .from_ = 0U,
              .to_ = 2U},
      booking{.r_ = {wish::kYes, wish::kAny, wish::kAny, wish::kAny},
              .from_ = 0U,
              .to_ = 2U},
      booking{.r_ = {wish::kYes, wish::kNo, wish::kAny, wish::kAny},
              .from_ = 1U,
              .to_ = 2U}};

  std::map<std::pair<station_id_t, reservation>, node_id_t> source_nodes,
      sink_nodes;

  auto const create_node = [&](station_id_t const s,
                               reservation const booking_r, bool const source) {
    return [&, booking_r, s, source]() {
      auto const node_id = static_cast<node_id_t>(nodes_.size());
      nodes_.emplace_back(node{.r_ = booking_r, .e_ = 0U, .station_ = s});
      out_edges_.resize(nodes_.size());
      in_edges_.resize(nodes_.size());

      for (auto const& [node_r, n] : station_nodes[s]) {
        if (matches(booking_r, node_r)) {
          out_edges_[source ? node_id : n].emplace_back(
              edge{.target_ = source ? n : node_id,
                   .capacity_ = std::numeric_limits<capacity_t>::max()});
          in_edges_[source ? n : node_id].emplace_back(
              edge{.target_ = source ? node_id : n,
                   .capacity_ = std::numeric_limits<capacity_t>::max()});
        }
      }

      return node_id;
    };
  };

  for (auto const& b : bookings) {
    auto const source_node_id =
        utl::get_or_create(source_nodes, std::pair{b.from_, b.r_},
                           create_node(b.from_, b.r_, true));
    auto const sink_node_id = utl::get_or_create(
        sink_nodes, std::pair{b.to_, b.r_}, create_node(b.to_, b.r_, false));

    ++nodes_[source_node_id].e_;
    --nodes_[sink_node_id].e_;
  }

  cista::raw::matrix<capacity_t> cap_matrix;
  cap_matrix.resize(nodes_.size(), nodes_.size());
  cap_matrix.reset(0U);
  for (auto const& [from_node_id, out_edges] : utl::enumerate(out_edges_)) {
    for (auto const& e : out_edges) {
      cap_matrix[from_node_id][e.target_] = e.capacity_;
    }
  }

  auto solver =
      std::unique_ptr<gor::MPSolver>{gor::MPSolver::CreateSolver("SCIP")};
  if (!solver) {
    LOG(WARNING) << "SCIP solver unavailable.";
    return -1;
  }

  std::map<std::pair<node_id_t, node_id_t>, gor::MPVariable*> flows;
  auto const get_flow_variable = [&](node_id_t const from, node_id_t const to) {
    return utl::get_or_create(flows, std::pair{from, to}, [&]() {
      return solver->MakeIntVar(
          0.0, cap_matrix[from][to],
          fmt::format("flow_{}_to_{}", nodes_[from], nodes_[to]));
    });
  };

  for (auto node_id = 0U; node_id != nodes_.size(); ++node_id) {
    auto const flow_conservation =
        solver->MakeRowConstraint(nodes_[node_id].e_, nodes_[node_id].e_);

    for (auto i = 0U; i != out_edges_[node_id].size(); ++i) {
      auto const to = out_edges_[node_id][i].target_;
      flow_conservation->SetCoefficient(get_flow_variable(node_id, to), 1);
    }

    for (auto i = 0U; i != in_edges_[node_id].size(); ++i) {
      auto const from = in_edges_[node_id][i].target_;
      flow_conservation->SetCoefficient(get_flow_variable(from, node_id), -1);
    }
  }

  auto const result_status = solver->Solve();
  std::cout << "result status: " << result_status << "\n";
  std::cout << "values:\n";
  for (auto const& [k, v] : flows) {
    auto const [from, to] = k;
    std::cout << "  " << nodes_[from] << " -> " << nodes_[to] << ": "
              << v->solution_value() << "\n";
  }

  std::cout << "digraph R {\n";
  std::cout << "  node [shape=record]\n";
  std::cout << "  rankdir = LR;\n";

  for (auto const& [station_id, r_n] : utl::enumerate(station_nodes)) {
    std::cout << "  { rank=same ";
    for (auto const& [r, n] : r_n) {
      std::cout << "  " << nodes_[n] << " ";
    }
    std::cout << "}\n";
  }

  std::cout << "  { rank=source ";
  for (auto const& [s_r, node_id] : source_nodes) {
    auto const& [station_id, r] = s_r;
    std::cout << nodes_[node_id] << " ";
  }
  std::cout << "}\n";

  std::cout << "  { rank=sink ";
  for (auto const& [s_r, node_id] : sink_nodes) {
    auto const& [station_id, r] = s_r;
    std::cout << nodes_[node_id] << " ";
  }
  std::cout << "}\n";

  for (auto const& [station_id, r_n] : utl::enumerate(station_nodes)) {
    for (auto const& [r, n] : r_n) {
      std::cout << "  " << nodes_[n] << " [label = \""
                << static_cast<char>('A' + station_id) << " " << r << "\"];\n";
    }
  }

  for (auto const& [s_r, node_id] : source_nodes) {
    auto const& [station_id, r] = s_r;
    std::cout << "  " << nodes_[node_id] << " [label = \""
              << static_cast<char>('A' + station_id) << " " << r << " "
              << nodes_[node_id].e_ << "\"  color=\"green\"];\n";
  }

  for (auto const& [s_r, node_id] : sink_nodes) {
    auto const& [station_id, r] = s_r;
    std::cout << "  " << nodes_[node_id] << " [label = \""
              << static_cast<char>('A' + station_id) << " " << r << " "
              << nodes_[node_id].e_ << "\" color=\"red\"];\n";
  }

  for (auto const& [node_id, edges] : utl::enumerate(out_edges_)) {
    auto const& from = nodes_[node_id];
    for (auto const& e : edges) {
      auto const& to = nodes_[e.target_];
      std::cout << "  " << from << " -> " << to;

      std::cout << " [label=\"";
      auto const flow = flows.find(std::pair{node_id, e.target_});
      if (flow != end(flows)) {
        std::cout << flow->second->solution_value();
      } else {
        std::cout << "-";
      }

      if (e.capacity_ != std::numeric_limits<capacity_t>::max()) {
        std::cout << " / " << e.capacity_;
      }
      std::cout << "\"];\n";
    }
  }

  constexpr auto const print_in_edges = false;
  if (print_in_edges) {
    for (auto const& [node_id, edges] : utl::enumerate(in_edges_)) {
      auto const& from = nodes_[node_id];
      for (auto const& e : edges) {
        auto const& to = nodes_[e.target_];
        std::cout << "  " << from << " -> " << to << " [";
        if (e.capacity_ != std::numeric_limits<capacity_t>::max()) {
          std::cout << "label=\"" << e.capacity_ << "\" ";
        }
        std::cout << "style=dashed];\n";
      }
    }
  }

  std::cout << "}\n\n\n";
}
