#pragma once

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"

namespace seat {

struct solver;

enum class node_type { kSegment, kSource, kSink };

struct flow_graph {
  struct node {
    friend std::ostream& operator<<(std::ostream& out, node const& n);
    reservation r_;
    std::int32_t e_;
    station_id_t station_;
    node_type type_{node_type::kSegment};
  };

  struct edge {
    node_id_t target_{0U};
    capacity_t capacity_{0U};
    capacity_t flow_{0U};
  };

  flow_graph(std::map<reservation, std::uint32_t> const& number_of_seats,
             unsigned number_of_segments);
  ~flow_graph();

  bool solve();
  void add_booking(booking const&);
  void to_graphviz(std::ostream& out, bool print_in_edges);
  std::string lp_str() const;

  auto get_create_node_fn(station_id_t, reservation booking_r, bool is_source);
  capacity_t get_capacity(node_id_t from, node_id_t to);
  std::optional<edge*> get_edge(node_id_t from, node_id_t to);

  std::vector<node> nodes_;
  std::vector<std::vector<edge>> out_edges_;
  std::vector<std::vector<edge>> in_edges_;
  std::vector<std::map<reservation, node_id_t>> station_nodes_;
  std::map<std::pair<station_id_t, reservation>, node_id_t> source_nodes_,
      sink_nodes_;
  std::unique_ptr<solver> solver_;
};

}  // namespace seat
