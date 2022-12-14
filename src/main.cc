#include <array>
#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <vector>

#include "utl/zip.h"

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

struct node {
  node(reservation r, std::size_t total, std::vector<node*> out)
      : r_{r}, total_{total}, out_{std::move(out)} {
    for (auto& o : out_) {
      o->in_.emplace_back(this);
    }
  }

  reservation r_;
  std::size_t total_, reserved_{0U}, reserved_out_{0U};
  bool available_{total_ != 0U};
  std::vector<node*> in_, out_;
};

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

template <typename Fn>
void for_each_subnode(std::vector<node*> const& out, Fn&& f) {
  std::set<node const*> visited;
  std::queue<node const*> q;
  for (auto const* out_n : out) {
    q.emplace(out_n);
    visited.emplace(out_n);
  }
  while (!q.empty()) {
    auto top = q.front();
    q.pop();
    for (auto const next : top->out_) {
      auto const is_new = visited.emplace(next).second;
      if (is_new) {
        q.emplace(next);
      }
    }
  }

  for (auto const& n : visited) {
    f(n);
  }
}

std::size_t compute_total(std::vector<node*> const& out) {
  std::size_t total = 0U;
  for_each_subnode(out, [&](node const* sub_n) {
    if (is_concrete(sub_n->r_)) {
      total += sub_n->total_;
    }
  });
  return total;
}

int main() {
  std::vector<std::unique_ptr<node>> node_mem;
  auto const build_node = [&](reservation r, std::size_t total,
                              std::vector<node*> out = {}) {
    return node_mem
        .emplace_back(std::make_unique<node>(r, total, std::move(out)))
        .get();
  };

  std::array<std::vector<node*>, kNProperties + 1> layers;
  layers[0] = {
      build_node({wish::kYes, wish::kNo, wish::kNo, wish::kYes}, 4),
      build_node({wish::kNo, wish::kNo, wish::kNo, wish::kYes}, 4),
      build_node({wish::kYes, wish::kYes, wish::kNo, wish::kYes}, 4),
      build_node({wish::kNo, wish::kYes, wish::kNo, wish::kYes}, 4),
  };

  auto const build_layer = [&](std::size_t const layer_idx) {
    auto& curr_layer = layers[layer_idx];
    auto& prev_layer = layers[layer_idx - 1];
    for (auto& e : prev_layer) {
      for (auto p = 0U; p != kNProperties; ++p) {
        if (e->r_[p] != wish::kAny) {
          std::vector<node*> out;

          auto copy = e->r_;
          copy[p] = wish::kAny;

          auto const exists = std::find_if(begin(curr_layer), end(curr_layer),
                                           [&](node const* n) {
                                             return n->r_ == copy;
                                           }) != end(curr_layer);
          if (exists) {
            continue;
          }

          for (auto& e1 : prev_layer) {
            if (matches(copy, e1->r_)) {
              out.emplace_back(e1);
            }
          }

          auto const total = compute_total(out);

          curr_layer.emplace_back(build_node(copy, total, std::move(out)));
        }
      }
    }
  };

  for (std::size_t i = 1; i != layers.size(); ++i) {
    build_layer(i);
  }

  std::cout << "digraph R {\n";
  std::cout << "  node [shape=record]\n";
  for (auto const& l : layers) {
    std::cout << "  { rank=same ";
    for (auto const& x : l) {
      std::cout << x->r_ << " ";
    }
    std::cout << "}\n";
  }

  for (auto const& l : layers) {
    for (auto const& x : l) {
      std::cout << "  " << x->r_ << " [label = \"" << x->r_ << " " << x->total_
                << "\"];\n";
    }
  }

  std::cout << "\n";

  for (auto const& l : layers) {
    for (auto const& x : l) {
      for (auto const& out : x->out_) {
        std::cout << "  " << x->r_ << " -> " << out->r_ << ";\n";
      }
    }
  }

  std::cout << "}\n";
}
