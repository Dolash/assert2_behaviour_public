#include "assert2_behaviour/assert2_behaviour.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "assert2_behaviour");
  ros::NodeHandle nh;

  Assert2Behaviour assert2Behaviour(nh);

  try {
    assert2Behaviour.spin();
  }
  catch (std::runtime_error& ex) {
    ROS_FATAL_STREAM("[ASSERT2_BEHAVIOUR] Runtime error: " << ex.what());
    return 1;
  }

  return 0;
}
