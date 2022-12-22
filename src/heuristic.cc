#include "seat/heuristic.h"

#include <cassert>
#include <algorithm>
#include <limits>
#include <stack>

#include "seat/booking.h"
#include "seat/flow_graph.h"
#include "utl/verify.h"

namespace seat {

template <typename Fn>
bool for_each_edge(flow_graph& g, flow_graph::edge& first_edge,
                   station_id_t const to, node_id_t const last_node, Fn&& fn) {
  {
    auto const continue_run = fn(first_edge);
    if (!continue_run) {
      return false;
    }
  }

  node_id_t curr = first_edge.target_;
  while (true) {
    if (g.nodes_[curr].station_ == to) {
      break;
    }

    auto const next =
        std::find_if(begin(g.out_edges_[curr]), end(g.out_edges_[curr]),
                     [&](flow_graph::edge const& e) {
                       return g.nodes_[e.target_].type_ == node_type::kSegment;
                     });
    if (next == end(g.out_edges_[curr])) {
      return false;
    }

    auto const continue_run = fn(*next);
    if (!continue_run) {
      return false;
    }

    curr = next->target_;
  }

  {
    auto const next = std::find_if(
        begin(g.out_edges_[curr]), end(g.out_edges_[curr]),
        [&](flow_graph::edge const& e) { return e.target_ == last_node; });
    utl::verify(next != end(g.out_edges_[curr]),
                "segment node at destination station {} has no connection to "
                "reservation target node {}",
                g.nodes_[curr], g.nodes_[last_node]);
    auto const continue_run = fn(*next);
    if (!continue_run) {
      return false;
    }
  }

  return true;
}

void increment_flow(flow_graph& g, flow_graph::edge& first_edge,
                    station_id_t const to, node_id_t const sink_node_id) {
  for_each_edge(g, first_edge, to, sink_node_id, [&](flow_graph::edge& e) {
    ++e.flow_;
    return true;
  });
}

capacity_t check_path(flow_graph& g, flow_graph::edge& first_edge,
                      station_id_t const to, node_id_t const sink_node_id) {
  auto min_remaining_capacity = std::numeric_limits<capacity_t>::max();
  auto const destination_reached = for_each_edge(
      g, first_edge, to, sink_node_id, [&](flow_graph::edge const& e) {
        assert(e.flow_ <= e.capacity_);
        min_remaining_capacity =
            std::min(e.capacity_ - e.flow_, min_remaining_capacity);
        return min_remaining_capacity != 0U;
      });
  if (!destination_reached) {
    return 0U;
  }
  return min_remaining_capacity;
}

bool heuristic(flow_graph& g, booking const& b) {
  auto const src_node_id = g.source_nodes_.at(std::pair{b.from_, b.r_});
  auto const sink_node_id = g.sink_nodes_.at(std::pair{b.to_, b.r_});
  auto max_min_remaining_capacity = 0U;
  flow_graph::edge* best_edge = nullptr;
  for (auto& first_edge : g.out_edges_[src_node_id]) {
    auto const min_remaining_capacity =
        check_path(g, first_edge, b.to_, sink_node_id);
    if (min_remaining_capacity > max_min_remaining_capacity) {
      max_min_remaining_capacity = min_remaining_capacity;
      best_edge = &first_edge;
    }
  }

  if (max_min_remaining_capacity != 0U) {
    increment_flow(g, *best_edge, b.to_, sink_node_id);
  }

  return max_min_remaining_capacity != 0U;
}

}  // namespace seat
