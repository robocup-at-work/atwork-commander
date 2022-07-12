#include <atwork_commander_gen/TaskGenerator.h>

#include <gtest/gtest.h>

namespace atwork_commander {
namespace task_generator {
namespace jurek {

class JurekGenSuite : public ::testing::Test  {
  private:

  protected:
};

TEST_F(JurekGenSuite, init) {
    TaskGenerator Gen("test_arena", "test_tasks", "jurek");

}


}
}
}
