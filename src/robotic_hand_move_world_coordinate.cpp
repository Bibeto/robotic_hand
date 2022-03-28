#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream> 

using namespace std ; 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotic_hand_move_world_coordinate");
  ros::NodeHandle n;

  int joint_number ; 
  std::cout << "Enter the joint number " << std::endl ; 
  std::cin >> joint_number ; 
  std::stringstream total_topic ; 
  total_topic << "/robotic_hand/joint" << joint_number << "_position_controller/command" ; 
     

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>(total_topic.str(), 1000);
  
  std::cout << "Give the extent of movement" << std::endl ; 
  double extent_of_mvt = 0.0 ; 
  std::cin >> extent_of_mvt ; 
 

  std_msgs::Float64 instruction; 
  instruction.data = extent_of_mvt ; 

  chatter_pub.publish(instruction);

  ros::spinOnce();

   

  return 0;
}