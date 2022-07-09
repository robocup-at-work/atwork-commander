#pragma once

#include <ros/ros.h>

#include <atwork_commander/Control.hpp>

#include <gtest/gtest.h>

#include <string>
#include <functional>

namespace atwork_commander {
namespace testing {

struct BasicControlTest : public ::testing::Test {

  Control control;

  static std::string readRefboxName() {
    std::string refbox;
    if( !ros::param::get("~refbox", refbox) ) {
        refbox = "atwork_commander";
        ROS_WARN_STREAM("[TEST-SETUP] No Refbox name specified! Using \"" << refbox << "\"!");
    }
    return refbox;
  }

  void SetUp() override {
    control.refbox(readRefboxName());
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

#define EXPECT_STATE_CALL(targetState)                                        \
  EXPECT_CALL(refbox,                                                         \
    stateChange(                                                              \
      Field( &StateUpdate::Request::state,                                    \
        Eq( targetState )                                                     \
      ),                                                                      \
      _                                                                       \
    )                                                                         \
  ).Times(1)                                                                  \
   .WillOnce(                                                                 \
    Return( true )                                                            \
  );
