#include "BasicControlTest.hpp"

#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>

#include <gmock/gmock.h>

#include <atwork_commander_msgs/LoadTask.h>
#include <atwork_commander_msgs/GenerateTask.h>
#include <atwork_commander_msgs/StartTask.h>
#include <atwork_commander_msgs/StateUpdate.h>

#include <thread>
#include <mutex>
#include <atomic>

using namespace std;
namespace p = placeholders;
using namespace atwork_commander;
using namespace atwork_commander_msgs;
using AdSrvOpts = ros::AdvertiseServiceOptions;
using ::testing::Return;
using ::testing::_;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::Field;
using ::testing::Eq;
using ::testing::IsEmpty;
using ::testing::SizeIs;
using ::testing::AllOf;


struct RefboxMockBase {
  class RefboxWrapper {
    RefboxState mState;
    mutex mMutex;
    public:
      RefboxWrapper& operator=(const RefboxState& state) { 
        lock_guard<mutex> lock(mMutex); 
        mState = state;
        return *this;
      }
      RefboxState load() { lock_guard<mutex> lock(mMutex); return mState; }
  } state;
  ros::Publisher statePub;
  ros::CallbackQueue queue;
  ros::ServiceServer loadSrv, storeSrv, stateChangeSrv, startSrv, generateSrv;
  ros::Timer timer;
  atomic<bool> stopped;
  thread run;

  virtual bool loadTask(LoadTask::Request&, LoadTask::Response&) = 0;
  virtual bool stateChange(StateUpdate::Request&, StateUpdate::Response&) = 0;
  virtual bool start(StartTask::Request&, StartTask::Response&) = 0;
  virtual bool generate(GenerateTask::Request&, GenerateTask::Response&) = 0;

  void periodic(const ros::TimerEvent& e) {
    statePub.publish(state.load());
  }

  bool loadTaskCall(LoadTask::Request& req, LoadTask::Response& res) { return loadTask(req, res); }
  bool startCall(StartTask::Request& req, StartTask::Response& res) { return start(req, res); }
  bool stateChangeCall(StateUpdate::Request& req, StateUpdate::Response& res) { return stateChange(req, res); }
  bool generateCall(GenerateTask::Request& req, GenerateTask::Response& res) { return generate(req, res); }

  RefboxMockBase()
    : stopped(false),
      run([this](){ while( !stopped.load() ) queue.callAvailable(ros::WallDuration(1)); } )
  {}

  void connect(const string& refbox) {
    ros::NodeHandle nh;
    statePub = nh.advertise<RefboxState>(refbox + "/internal/state", 1);

    auto loadSrvOpts = AdSrvOpts::create<LoadTask>( refbox + "/internal/load_task",
                                                    std::bind(&RefboxMockBase::loadTaskCall, this, p::_1, p::_2),
                                                    ros::VoidConstPtr(),
                                                    &queue 
                                                  );
    loadSrv = nh.advertiseService( loadSrvOpts );
    
    auto scSrvOpts = AdSrvOpts::create<StateUpdate>( refbox + "/internal/state_update",
                                                     std::bind(&RefboxMockBase::stateChangeCall, this, p::_1, p::_2),
                                                     ros::VoidConstPtr(),
                                                     &queue
                                                   );
    stateChangeSrv = nh.advertiseService( scSrvOpts );

    auto startSrvOpts = AdSrvOpts::create<StartTask>( refbox + "/internal/start_task",
                                                      std::bind(&RefboxMockBase::startCall, this, p::_1, p::_2), 
                                                      ros::VoidConstPtr(),
                                                      &queue
                                                    );
    startSrv = nh.advertiseService( startSrvOpts );

    auto genSrvOpts = AdSrvOpts::create<GenerateTask>( refbox + "/internal/generate_task",
                                                       std::bind(&RefboxMockBase::generateCall, this, p::_1, p::_2),
                                                       ros::VoidConstPtr(),
                                                       &queue
                                                     );
    generateSrv = nh.advertiseService( genSrvOpts );

    auto timOpts = ros::TimerOptions( ros::Duration(1), std::bind(&RefboxMockBase::periodic, this, p::_1), &queue );

    timer = nh.createTimer( timOpts );
  }

  void disconnect() {
    stopped = true;
    timer.stop();
    statePub.shutdown();
    loadSrv.shutdown();
    startSrv.shutdown();
    generateSrv.shutdown();
    stateChangeSrv.shutdown();
  }

  virtual ~RefboxMockBase() { stopped = true; run.join(); }

};

struct RefboxMock : public RefboxMockBase {
#ifndef MOCK_METHOD2
  MOCK_METHOD(bool, loadTask, (LoadTask::Request&, LoadTask::Response&), (override));
  MOCK_METHOD(bool, start, (StartTask::Request&, StartTask::Response&), (override));
  MOCK_METHOD(bool, generate, (GenerateTask::Request&, GenerateTask::Response&), (override));
  MOCK_METHOD(bool, stateChange, (StateUpdate::Request&, StateUpdate::Response&), (override));
#else
  MOCK_METHOD2(loadTask, bool(LoadTask::Request&, LoadTask::Response&));
  MOCK_METHOD2(start, bool(StartTask::Request&, StartTask::Response&));
  MOCK_METHOD2(generate, bool(GenerateTask::Request&, GenerateTask::Response&));
  MOCK_METHOD2(stateChange, bool(StateUpdate::Request&, StateUpdate::Response&));
#endif
};

struct ControlTests : public atwork_commander::testing::BasicControlTest {

  RefboxMock refbox;

  void SetUp() override {
    atwork_commander::testing::BasicControlTest::SetUp();
    refbox.connect(readRefboxName());
  }
  
  void TearDown() override {
    atwork_commander::testing::BasicControlTest::TearDown();
    refbox.disconnect();
  }

  void toState(unsigned int next) {
    RefboxState state;
    state.state = next;
    refbox.state = state;
    while( control.state().state != next)
      ros::spinOnce();
  }

};

TEST_F(ControlTests, forwardFromFail) {
  toState(RefboxState::FAILURE);

  EXPECT_REASON(control.forward(), STATE_INVALID);
}

TEST_F(ControlTests, forwardFromIdle) {
  toState(RefboxState::IDLE);

  EXPECT_REASON(control.forward(), STATE_INVALID);
}

TEST_F(ControlTests, forwardFromReady) {
  toState(RefboxState::READY);

  EXPECT_REASON(control.forward(), STATE_INVALID);
}

TEST_F(ControlTests, forwardFromPrep) {
  toState(RefboxState::PREPARATION);

  EXPECT_STATE_CALL( RefboxState::EXECUTION );

  EXPECT_NO_THROW(control.forward());
}


TEST_F(ControlTests, forwardFromExec) {
  toState(RefboxState::EXECUTION);

  EXPECT_STATE_CALL( RefboxState::READY );

  EXPECT_NO_THROW(control.forward());
}

TEST_F(ControlTests, stopFromIdle) {
  toState(RefboxState::IDLE);

  EXPECT_STATE_CALL( RefboxState::IDLE );

  EXPECT_NO_THROW(control.stop());
}

TEST_F(ControlTests, stopFromReady) {
  toState(RefboxState::READY);

  EXPECT_STATE_CALL( RefboxState::READY );

  EXPECT_NO_THROW(control.stop());
}

TEST_F(ControlTests, stopFromPrep) {
  toState(RefboxState::PREPARATION);

  EXPECT_STATE_CALL( RefboxState::READY );

  EXPECT_NO_THROW(control.stop());
}

TEST_F(ControlTests, stopFromExec) {
  toState(RefboxState::EXECUTION);

  EXPECT_STATE_CALL( RefboxState::READY );

  EXPECT_NO_THROW(control.stop());
}

TEST_F(ControlTests, stopFromFail) {
  toState(RefboxState::FAILURE);

  EXPECT_REASON(control.stop(), STATE_INVALID);
}

TEST_F(ControlTests, forwardServiceError) {
  toState(RefboxState::PREPARATION);

  StateUpdate::Response res;
  res.error = "Something";

  EXPECT_CALL(refbox,
    stateChange(
      _,
      _
    )
  ).Times(1)
   .WillOnce(
     DoAll(
       SetArgReferee<1>( res ),
       Return( true )
     )
   );
  EXPECT_REASON(control.forward(), SERVICE_ERROR);
}

TEST_F(ControlTests, startFromFail) {
  toState(RefboxState::FAILURE);

  EXPECT_REASON(control.start(), STATE_INVALID);
}

TEST_F(ControlTests, startFromIdle) {
  toState(RefboxState::IDLE);

  EXPECT_REASON(control.start(), STATE_INVALID);
}

TEST_F(ControlTests, startFromReady) {
  toState(RefboxState::READY);

  EXPECT_CALL(refbox,
    start(
      Field( &StartTask::Request::robots,
        IsEmpty()
      ),
      _
    )
  ).Times(1)
   .WillOnce(
     Return( true )
   );

  EXPECT_NO_THROW(control.start());
}

TEST_F(ControlTests, startFromPrep) {
  toState(RefboxState::PREPARATION);

  EXPECT_REASON(control.start(), STATE_INVALID);
}


TEST_F(ControlTests, startFromExec) {
  toState(RefboxState::EXECUTION);

  EXPECT_REASON(control.start(), STATE_INVALID);
}

TEST_F(ControlTests, startServiceError) {
  toState(RefboxState::READY);

  StartTask::Response res;
  res.error = "Something";

  EXPECT_CALL(refbox, start(_, _))
    .Times(1)
    .WillOnce( DoAll( SetArgReferee<1>(res), Return(true) ) );
  EXPECT_REASON(control.start(), SERVICE_ERROR);
}

TEST_F(ControlTests, startSpecificRobot) {
  toState(RefboxState::READY);

  EXPECT_CALL(refbox,
    start(
      Field(&StartTask::Request::robots,
        AllOf(
          Contains(
            AllOf(
              Field( &RobotHeader::team_name, Eq("test")),
              Field( &RobotHeader::robot_name, Eq("test"))
            )
          ),
          SizeIs(1)
        )
      ),
      _
    )
  ).Times(1)
   .WillOnce(
     Return( true)
   );

  EXPECT_NO_THROW(control.start({"test/test"}));
}

TEST_F(ControlTests, startMultipleRobots) {
  toState(RefboxState::READY);

  EXPECT_CALL(refbox,
    start(
      Field(&StartTask::Request::robots,
        AllOf(
          Contains(
            AllOf(
              Field( &RobotHeader::team_name, Eq("test")),
              Field( &RobotHeader::robot_name, Eq("test"))
            )
          ),
          Contains(
            AllOf(
              Field( &RobotHeader::team_name, Eq("test")),
              Field( &RobotHeader::robot_name, Eq("test2"))
            )
          ),
          SizeIs(2)
        )
      ),
      _
    )
  ).Times(1)
   .WillOnce(
     Return(true)
   );

  EXPECT_NO_THROW(control.start({"test/test", "test/test2"}));
}

TEST_F(ControlTests, startInvalidRobot) {
  toState(RefboxState::READY);

  EXPECT_REASON(control.start({"test"}), ARGUMENT_INVALID);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_test");

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
