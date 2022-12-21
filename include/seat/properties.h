#pragma once

#include <type_traits>

namespace seat {

enum class properties { kWindow, kSilent, kTable, kBig, kNProperties };

constexpr auto const kNProperties =
    static_cast<std::underlying_type_t<properties>>(properties::kNProperties);

}  // namespace seat
