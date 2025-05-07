#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "include/rosbridge.h"
#include "include/can.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_chassis");
  printf("hello 13\n");

//   ros::NodeHandle n;

//   ros::Rate loop_rate(10);

  int count = 0;

  // std::shared_ptr<Rosbridge> rosToCan = std::make_shared<Rosbridge>();
  std::shared_ptr<control_can::Can> can = std::make_shared<control_can::Can>();
  if (can->Init() == -1) {
    std::cout<< " Init() failed!";
  }

  if (can->Start() == -1) {
    std::cout<< " Start() failed!";
  }

  // if (!can->readMileageConfigParameter())
  // {
  //   std::cerr << "Failed to read initial configuration!" << std::endl;
  //   return 1;
  // }

  // auto start = std::chrono::steady_clock::now();

  // while (true)
  // {
  //   sleep(5); // 等待5秒

  //   if (!can->writeMileageConfigParameter())
  //   {
  //     std::cerr << "Failed to write configuration!" << std::endl;
  //   }

  //   auto now = std::chrono::steady_clock::now();
  //   auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

  //   if (elapsed >= 30)
  //   {
  //     std::cout << "30 seconds elapsed, exiting..." << std::endl;
  //     break;
  //   }
  // }

  return 0;
}
