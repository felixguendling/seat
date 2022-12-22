#include "gtest/gtest.h"

#include "google/protobuf/stubs/common.h"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  auto test_result = RUN_ALL_TESTS();
  google::protobuf::ShutdownProtobufLibrary();
  return test_result;
}
