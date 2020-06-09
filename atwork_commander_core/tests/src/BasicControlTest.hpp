#pragma once

#include <ros/ros.h>

#include <atwork_commander/Control.hpp>

#include <gtest/gtest.h>

#include <string>
#include <functional>

namespace atwork_commander {
namespace testing {

  struct BasicControlTest : public ::testing::Test, public Control {

    static std::string readRefboxName() {
      std::string refbox;
      if( !ros::param::get("~refbox", refbox) ) {
          refbox = "atwork_commander";
          ROS_WARN_STREAM("[TEST-SETUP] No Refbox name specified! Using \"" << refbox << "\"!");
      }
      return refbox;
    }

    BasicControlTest(bool check = true) : Control(readRefboxName()) {
      while(check && state().state == RefboxState::FAILURE)
        ros::spinOnce();
    }

    void expectReason(std::function<void()> func, ControlError::Reasons reason) {
      try {
        func();
      } catch(const ControlError& e) {
        EXPECT_EQ(e.reason(), reason);
        throw;
      }
    }
  };
}
}

#define EXPECT_REASON(func, reason)                                           \
  EXPECT_THROW( expectReason( [ this ](){ func; },                            \
                              atwork_commander::ControlError::Reasons::reason \
                            ),                                                \
                atwork_commander::ControlError                                \
              );
