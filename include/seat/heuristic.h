#pragma once

namespace seat {

struct flow_graph;
struct booking;

bool heuristic(flow_graph&, booking const&);

}  // namespace seat
