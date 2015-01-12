/**
 * Large Map 
 * 
 * Camera based learning and navigating jockey.
 *
 */

#include <string>

#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <anj_featurenav/jockey.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "anj_featurenav");
  ros::NodeHandle private_nh("~");
  
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  std::string jockey_base_name;
  std::string default_jockey_base_name = ros::this_node::getName();
  private_nh.param<std::string>("jockey_base_name", jockey_base_name, default_jockey_base_name);

  anj_featurenav::Jockey jockey(jockey_base_name);

  ROS_INFO_STREAM(ros::this_node::getName() << " started (with servers " <<
      jockey.getLearningJockeyName() << " and " << jockey.getNavigatingJockeyName() << ")");
  ros::spin();
  return 0;
}

