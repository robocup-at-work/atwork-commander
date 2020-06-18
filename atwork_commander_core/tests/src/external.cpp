#include "BasicControlTest.hpp"

using External = atwork_commander::testing::BasicControlTest;

TEST_F(External, startOnIdle) {
  EXPECT_REASON(control.start(), STATE_INVALID);
}

TEST_F(External, forwardOnIdle) {
  EXPECT_REASON(control.forward(), STATE_INVALID);
}

TEST_F(External, stopOnIdle) {
  EXPECT_NO_THROW(control.stop());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "external_test");

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
