#include <atwork_commander_gen/TaskGenerator.h>

#include <ros/ros.h>

#include <gtest/gtest.h>

namespace atwork_commander {
namespace task_generator {
namespace jurek {

class JurekGenSuite : public ::testing::Test  {
  private:

  protected:
    TaskGenerator gen;
    JurekGenSuite(): gen("test_arena", "test_tasks", "jurek") {}
};

TEST_F(JurekGenSuite, tables) {
    auto task = gen("tables");
    ASSERT_TRUE(gen.check(task)) << task;
}

TEST_F(JurekGenSuite, specialTables) {
    auto task = gen("specialTables");
    ASSERT_TRUE(gen.check(task)) << task;
}


}
}
}
