#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mcf_solver_i.h"
#include "seat/booking.h"
#include "seat/reservation.h"
#include "seat/types.h"
#include "solver.h"

namespace seat {

struct flow_graph_solver;

enum class node_type { kSegment, kSource, kSink };

struct flow_graph : mcf_solver_i {
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
  flow_graph(flow_graph const&);
  ~flow_graph();

  bool solve();
  void add_booking(booking const&);
  void add_booking(booking const&, bool const);
  void remove_booking(booking const&);
  void remove_booking(booking const&, bool const);
  void to_graphviz(std::ostream& out, bool print_in_edges) const;
  std::string lp_str() const;
  void print_sizes() const;
  void print_name() const;
  void print() const;

  cista::raw::vector_map<booking_id_t, booking> get_mcf_bookings();
  std::map<seat_id_t, std::vector<booking>> get_gsd_bookings();

  std::map<reservation, bool> gsd_request(interval const&);
  void add_gsd_booking(booking const&, uint32_t const&);
  std::vector<std::pair<node_id_t, node_id_t>> get_reached_capacity_ids(
      reservation const&);
  std::vector<bool> get_removable_bookings(
      booking const&, std::vector<std::pair<node_id_t, node_id_t>> const&);

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
  std::map<uint32_t, booking> gsd_bookings_;
  std::map<uint32_t, std::vector<booking>> pseudo_gsd_bookings_;
  std::vector<booking> mcf_bookings_;
};
}  // namespace seat