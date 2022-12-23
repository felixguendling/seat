#include "gtest/gtest.h"

#include "seat/booking.h"
#include "seat/nogo_cache.h"

using namespace seat;

TEST(nogo_cache, works) {
  auto const b1 = booking{.r_ = {}, .interval_ = {3, 10}};
  auto const b2 = booking{.r_ = {}, .interval_ = {4, 10}};
  auto const b3 = booking{.r_ = {}, .interval_ = {2, 10}};
  auto const b4 = booking{.r_ = {}, .interval_ = {3, 11}};
  auto const b5 = booking{.r_ = {}, .interval_ = {3, 4}};
  auto const b6 = booking{.r_ = {wish::kYes}, .interval_ = {3, 4}};

  nogo_cache c;
  EXPECT_FALSE(c.is_nogo(b1));

  c.add_entry(b1);

  EXPECT_TRUE(c.is_nogo(b1));
  EXPECT_FALSE(c.is_nogo(b2));
  EXPECT_TRUE(c.is_nogo(b3));
  EXPECT_TRUE(c.is_nogo(b4));

  c.add_entry(b2);
  c.add_entry(b5);
  EXPECT_FALSE(c.is_nogo(b6));
}
