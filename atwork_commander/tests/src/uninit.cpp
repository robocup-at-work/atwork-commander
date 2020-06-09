#include <ros/ros.h>

#include <gtest/gtest.h>

#include <atwork_commander/Control.hpp>

#include <string>

using namespace std;
using namespace atwork_commander;
using namespace atwork_commander_msgs;


struct UnInit : public ::testing::Test {

  Control control;

  static string readRefBoxName() {
    string refbox;
    if( !ros::param::get("~refbox", refbox) ) {
        refbox = "atwork_commander";
        ROS_WARN_STREAM("[TEST-SETUP] No Refbox name specified! Using \"" << refbox << "\"!");
    }
    return refbox;
  }

  UnInit() : control(readRefBoxName()) {
    while(control.state().state == RefboxState::FAILURE)
      ros::spinOnce();
  }

};

ostream& operator<<(ostream& os, const ControlError& e) {
  return os << "ControlError: " << e.what();
}

TEST_F(UnInit, start) {
  try {
    control.start();
  } catch(const ControlError& e) {
    EXPECT_EQ(e.reason(), ControlError::Reasons::SERVICE_ERROR) << "Wrong Reason";
  }
  FAIL() << "Start on unitialized refbox did not trigger exception";
}

TEST_F(UnInit, stop) {
  try {
    control.stop();
  } catch(const ControlError& e) {
    FAIL() << "Exception triggered on call of stop on uninitalized refbox: " << e.what();
  }
}

TEST_F(UnInit, forward) {
  try {
    control.forward();
  } catch(const ControlError& e) {
    EXPECT_EQ(e.reason(), ControlError::Reasons::STATE_INVALID) << "Wrong Reason";
  }
  FAIL() << "forward on unitialized refbox did not trigger exception";
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_setup");

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
