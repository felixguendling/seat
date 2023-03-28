#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ortools/linear_solver/linear_solver.h"

#include "seat/types.h"

namespace seat {

struct flow_graph;

struct solver {
  explicit solver(flow_graph& graph);
  operations_research::MPVariable* get_flow(node_id_t from, node_id_t to);

  void add_fccs();

  void set_excess(node_id_t node_id, std::int32_t e);
  void add_flow(node_id_t from, node_id_t to);
  void set_capacity(node_id_t from, node_id_t to, capacity_t);

  bool solve();
  bool feasible();
  std::string lp_str() const;
  void write_flows() const;
  void print_sizes() const;
  void print() const;

  int get_occupied_seats(node_id_t const, node_id_t const) const;

private:
  operations_research::MPConstraint* get_fcc(node_id_t node_id);

  flow_graph& graph_;
  std::unique_ptr<operations_research::MPSolver> solver_;
  std::map<std::pair<node_id_t, node_id_t>, operations_research::MPVariable*>
      flows_;
  std::vector<operations_research::MPConstraint*> node_fccs_;
  operations_research::MPSolver::ResultStatus result_;
};

}  // namespace seat