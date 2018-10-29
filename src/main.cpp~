#include "assertive_behaviour/assertive_behaviour.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "assertive_behaviour");
  ros::NodeHandle nh;

  AssertiveBehaviour assertiveBehaviour(nh);

  try {
    assertiveBehaviour.spin();
  }
  catch (std::runtime_error& ex) {
    ROS_FATAL_STREAM("[ASSERTIVE_BEHAVIOUR] Runtime error: " << ex.what());
    return 1;
  }

  return 0;
}
