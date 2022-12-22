#include "seat/flow_graph.h"

#include <algorithm>
#include <limits>
#include <ostream>

#include "utl/enumerate.h"
#include "utl/get_or_create.h"
#include "utl/timing.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"

#include "ortools/base/logging.h"
#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model.pb.h"
#include "ortools/sat/cp_model_solver.h"
#include "ortools/util/sorted_interval_list.h"

namespace gor = operations_research;
namespace sat = gor::sat;

namespace seat {

constexpr auto const solver_name = "SCIP";

struct flow_graph::solver {
  explicit solver(flow_graph& graph) : graph_{graph} {
    utl::verify(solver_ != nullptr, "{} solver unavailable", solver_name);
  }

  gor::MPVariable* get_flow(node_id_t const from, node_id_t const to) {
    return utl::get_or_create(flows_, std::pair{from, to}, [&]() {
      auto const capacity = graph_.get_capacity(from, to);
      auto const adjusted = capacity == std::numeric_limits<capacity_t>::max()
                                ? solver_->infinity()
                                : capacity;
      return solver_->MakeIntVar(
          0.0, adjusted,
          fmt::format("flow_{}_to_{}", graph_.nodes_[from], graph_.nodes_[to]));
    });
  }

  void add_fccs() {
    for (auto node_id = 0U; node_id != graph_.nodes_.size(); ++node_id) {
      auto const fcc = get_fcc(node_id);
      fcc->SetBounds(graph_.nodes_[node_id].e_, graph_.nodes_[node_id].e_);

      for (auto i = 0U; i != graph_.out_edges_[node_id].size(); ++i) {
        auto const to = graph_.out_edges_[node_id][i].target_;
        fcc->SetCoefficient(get_flow(node_id, to), 1);
      }

      for (auto i = 0U; i != graph_.in_edges_[node_id].size(); ++i) {
        auto const from = graph_.in_edges_[node_id][i].target_;
        fcc->SetCoefficient(get_flow(from, node_id), -1);
      }
    }
  }

  gor::MPConstraint* get_fcc(node_id_t const node_id) {
    node_fccs_.resize(
        std::max(node_fccs_.size(), static_cast<std::size_t>(node_id + 1)),
        nullptr);
    auto& fcc = node_fccs_[node_id];
    if (fcc == nullptr) {
      fcc = solver_->MakeRowConstraint(
          fmt::format("fcc_{}", graph_.nodes_[node_id]));
    }
    return fcc;
  }

  void set_excess(node_id_t const node_id, std::int32_t const e) {
    get_fcc(node_id)->SetBounds(e, e);
  }

  void add_flow(node_id_t const from, node_id_t const to) {
    auto const flow = get_flow(from, to);
    node_fccs_[from]->SetCoefficient(flow, 1);
    node_fccs_[to]->SetCoefficient(flow, -1);
  }

  void set_capacity(node_id_t const from, node_id_t const to,
                    capacity_t const c) {
    flows_.at(std::pair{from, to})->SetBounds(0.0, c);
  }

  bool solve() {
    //    for (auto const& [from_to, var] : flows_) {
    //      auto const [from, to] = from_to;
    //      auto const from_node = graph_.nodes_[from];
    //      auto const to_node = graph_.nodes_[to];
    //      if (from_node.type_ == node_type::kSource) {
    //        var->SetInteger(true);
    //        var->SetUB(from_node.e_);
    //      } else if (to_node.type_ == node_type::kSink) {
    //        var->SetInteger(true);
    //        var->SetUB(-to_node.e_);
    //      }
    //    }

    result_ = solver_->Solve();

    return feasible();
  }

  std::string lp_str() const {
    std::string s;
    solver_->ExportModelAsLpFormat(false, &s);
    return s;
  }

  bool feasible() {
    return result_ == gor::MPSolver::OPTIMAL ||
           result_ == gor::MPSolver::FEASIBLE;
  }

  flow_graph& graph_;
  std::unique_ptr<gor::MPSolver> solver_{
      gor::MPSolver::CreateSolver(solver_name)};
  std::map<std::pair<node_id_t, node_id_t>, gor::MPVariable*> flows_;
  std::vector<gor::MPConstraint*> node_fccs_;
  gor::MPSolver::ResultStatus result_{gor::MPSolver::NOT_SOLVED};
};

struct flow_graph::sat_solver {
  explicit sat_solver(flow_graph& g) : graph_{g} {}

  void clear() {
    builder_ = {};

    node_fccs_.clear();
    node_fccs_.resize(graph_.nodes_.size());

    flows_.clear();

    response_.Clear();
  }

  void build() {
    for (auto node_id = 0U; node_id != graph_.nodes_.size(); ++node_id) {
      auto& [l, r] = node_fccs_[node_id];

      r = graph_.nodes_[node_id].e_;

      for (auto i = 0U; i != graph_.out_edges_[node_id].size(); ++i) {
        auto const to = graph_.out_edges_[node_id][i].target_;
        l += get_flow(node_id, to);
      }

      for (auto i = 0U; i != graph_.in_edges_[node_id].size(); ++i) {
        auto const from = graph_.in_edges_[node_id][i].target_;
        l -= get_flow(from, node_id);
      }
    }
  }

  sat::IntVar get_flow(node_id_t const from, node_id_t const to) {
    return utl::get_or_create(flows_, std::pair{from, to}, [&]() {
      auto capacity = graph_.get_capacity(from, to);

      auto const from_node = graph_.nodes_[from];
      auto const to_node = graph_.nodes_[to];
      if (from_node.type_ == node_type::kSource) {
        capacity = from_node.e_;
      } else if (to_node.type_ == node_type::kSink) {
        capacity = -to_node.e_;
      }

      auto var = builder_.NewIntVar(gor::Domain{0U, capacity});
      if (auto const it = flow_hint_.find(std::pair{from, to});
          it != end(flow_hint_)) {
        builder_.AddHint(var, it->second);
      }

      return var;
    });
  }

  bool solve() {
    clear();
    build();
    for (auto const& [l, r] : node_fccs_) {
      builder_.AddEquality(l, r);
    }

    //    builder_.Build().PrintDebugString();

    response_ = sat::Solve(builder_.Build());

    for (auto const& [from_to, var] : flows_) {
      flow_hint_[from_to] = sat::SolutionIntegerValue(response_, var);
    }

    return response_.status() == sat::OPTIMAL ||
           response_.status() == sat::FEASIBLE;
  }

  std::uint32_t get_flow_value(std::uint32_t const from,
                               std::uint32_t const to) const {
    auto const it = flows_.find(std::pair{from, to});
    if (it != end(flows_)) {
      return sat::SolutionIntegerValue(response_, it->second);
    } else {
      return 0U;
    }
  }

  sat::CpModelBuilder builder_;
  sat::CpSolverResponse response_;
  std::map<std::pair<node_id_t, node_id_t>, sat::IntVar> flows_;
  std::map<std::pair<node_id_t, node_id_t>, capacity_t> flow_hint_;
  std::vector<std::pair<sat::LinearExpr, sat::LinearExpr>> node_fccs_;
  flow_graph& graph_;
};

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

capacity_t flow_graph::get_capacity(node_id_t const from,
                                    node_id_t const to) const {
  auto const it = std::find_if(begin(out_edges_[from]), end(out_edges_[from]),
                               [&](edge const& e) { return e.target_ == to; });
  if (it == end(out_edges_[from])) {
    return std::numeric_limits<capacity_t>::max();
  } else {
    return it->capacity_;
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

      if (solver_->feasible()) {
        auto const flow = solver_->flows_.find(std::pair{node_id, e.target_});
        if (flow != end(solver_->flows_)) {
          out << flow->second->solution_value();
        } else {
          out << "-";
        }
      }

      if (e.capacity_ != std::numeric_limits<capacity_t>::max()) {
        out << " / " << e.capacity_;
      }
      out << "\"];\n";

      if (sat_solver_ != nullptr) {
        out << "  " << from << " -> " << to;
        out << " [fontcolor=blue label=\""
            << sat_solver_->get_flow_value(node_id, e.target_) << "\"];\n";
      }
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
  if (sat_solver_ == nullptr) {
    sat_solver_ = std::make_unique<sat_solver>(*this);
  }

  UTL_START_TIMING(sat_t);
  auto const x = sat_solver_->solve();
  UTL_STOP_TIMING(sat_t);

  UTL_START_TIMING(milp_t);
  auto const y = solver_->solve();
  UTL_STOP_TIMING(milp_t);

  std::cout << "  sat_timing=" << UTL_GET_TIMING_MS(sat_t) << "\n";
  std::cout << "  milp_timing=" << UTL_GET_TIMING_MS(milp_t) << "\n";

  utl::verify(x == y, "sat_solver={}, milp_solver={}", x, y);

  return x;
}

std::string flow_graph::lp_str() const { return solver_->lp_str(); }

}  // namespace seat
