#include "seat/solver.h"

#include "utl/get_or_create.h"
#include "utl/verify.h"

#include "ortools/linear_solver/linear_solver.h"

#include "seat/flow_graph.h"

namespace seat {

constexpr auto const solver_name = "SCIP";

namespace gor = operations_research;

solver::solver(flow_graph& graph)
    : graph_{graph},
      solver_{gor::MPSolver::CreateSolver(solver_name)},
      result_{operations_research::MPSolver::NOT_SOLVED} {
  utl::verify(solver_ != nullptr, "{} solver unavailable", solver_name);
}

gor::MPVariable* solver::get_flow(node_id_t const from, node_id_t const to) {
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

void solver::add_fccs() {
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

gor::MPConstraint* solver::get_fcc(node_id_t const node_id) {
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

void solver::set_excess(node_id_t const node_id, std::int32_t const e) {
  get_fcc(node_id)->SetBounds(e, e);
}

void solver::add_flow(node_id_t const from, node_id_t const to) {
  auto const flow = get_flow(from, to);
  node_fccs_[from]->SetCoefficient(flow, 1);
  node_fccs_[to]->SetCoefficient(flow, -1);
}

void solver::set_capacity(node_id_t const from, node_id_t const to,
                          capacity_t const c) {
  flows_.at(std::pair{from, to})->SetBounds(0.0, c);
}

bool solver::solve() {
  std::vector<std::pair<gor::MPVariable const*, double>> hints;
  for (auto const& [from_to, var] : flows_) {
    auto const [from, to] = from_to;
    hints.emplace_back(var, (*graph_.get_edge(from, to))->flow_);
  }
  solver_->SetHint(hints);
  result_ = solver_->Solve();
  return feasible();
}

std::string solver::lp_str() const {
  std::string s;
  solver_->ExportModelAsLpFormat(false, &s);
  return s;
}

bool solver::feasible() {
  return result_ == gor::MPSolver::OPTIMAL ||
         result_ == gor::MPSolver::FEASIBLE;
}

void solver::write_flows() const {
  for (auto const [from_to, flow_var] : flows_) {
    auto const [from, to] = from_to;
    auto& edges = graph_.out_edges_[from];

    auto const it = std::find_if(
        begin(edges), end(edges),
        [&, to = to](flow_graph::edge const& e) { return e.target_ == to; });
    if (it != end(edges)) {
      it->flow_ = static_cast<capacity_t>(flow_var->solution_value());
    }
  }
}

}  // namespace seat
