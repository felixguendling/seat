#pragma once

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"

namespace seat {

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
    node_id_t target_;
    capacity_t capacity_;
  };

  flow_graph(std::map<reservation, std::uint32_t> const& number_of_seats,
             unsigned number_of_segments);
  ~flow_graph();

  bool solve();
  void add_booking(booking const&);
  void to_graphviz(std::ostream& out, bool print_in_edges);

private:
  auto get_create_node_fn(station_id_t const s, reservation const booking_r,
                          bool const is_sourc);
  capacity_t get_capacity(node_id_t from, node_id_t to) const;

  std::vector<node> nodes_;
  std::vector<std::vector<edge>> out_edges_;
  std::vector<std::vector<edge>> in_edges_;
  std::vector<std::map<reservation, node_id_t>> station_nodes_;
  std::map<std::pair<station_id_t, reservation>, node_id_t> source_nodes_,
      sink_nodes_;
  struct solver;
  std::unique_ptr<solver> solver_;
};

}  // namespace seat
