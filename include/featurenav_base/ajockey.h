/*
 * Learns a segment, i.e. a series of features with additional information,
 * with information based on a camera and odometry.
 *
 * Long description:
 * Learns a segment, i.e. a series of features with additional information,
 * with information based on a camera and odometry. A feature with additional
 * information is called a landmark. Implemented actions:
 * - START: will learn new features indefinitely or on at least a configurable
 *   distance.
 * - STOP: will interrupt the START action and save the segment.
 * - INTERRUPT: TODO
 * - CONTINUE: TODO
 *
 * Interaction with the map (created by this jockey):
 * - [Getter][/][Setter], message type, interface default name
 *
 * Interaction with the map (created by other jockeys):
 * - [Getter][/][Setter], message type, interface default name
 * - Setter: Segment, the interface name is given by the subclass
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - sensor_msgs/Image (raw), "~/camera/image_raw", image from a front camera.
 * - nav_msgs::Odometry, "~/odom", Odometry.
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 *
 * Services used (other than map-related):
 * - none
 *
 * Parameters:
 * - matcher_max_relative_distance, double, 0.8, a potential descriptor is
 *   visible in the new image if the distance to the best-match descriptor is
 *   smaller than the distance to the second best match multiplied by this factor.
 * - min_landmark_dist, double, 0.020 m, a landmark is saved if it was visible
 *   on at least such a traveled distance (m).
 * - max_segment_length, double, 0, the jockey will successfully end the START
 *   action if the traveled distance is greater than max_segment_length (m).
 *   Defaults to 0, which means that the START action never finishes.
 */

#ifndef FEATURENAV_BASE_AJOCKEY_H
#define FEATURENAV_BASE_AJOCKEY_H

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <lama_jockeys/learning_jockey.h>

#include <featurenav_base/potential_landmark.h>
#include <featurenav_base/typedef.h>
#include <featurenav_base/Landmark.h>
#include <featurenav_base/Segment.h>
#include <featurenav_base/SetSegment.h>

namespace featurenav_base {

using std::vector;
using cv::Mat;
using cv::KeyPoint;
using cv::DMatch;

class AJockey : public lama_jockeys::LearningJockey
{
  public:

    AJockey(const std::string& name, const std::string& segment_interface_name, const std::string& segment_setter_name);

    void setExtractFeaturesFunction(feature_extractor_function_ptr f) {extract_features_ = f;}
    void setDescriptorMatcherFunction(descriptor_matcher_function_ptr f) {match_descriptors_ = f;}

    can_do_function_ptr canDo;
    start_do_function_ptr startDo;

  private:

    virtual void onLearn();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

    void reset();
    size_t processImage(const sensor_msgs::ImageConstPtr& image);

    /* Return the distance since we started learning.
     */
    inline double distance_from_start()
    {
      if (!has_odom_)
      {
        return 0.0;
      }
      const double dx = odom_.pose.pose.position.x - start_pose_.position.x;
      const double dy = odom_.pose.pose.position.y - start_pose_.position.y;
      return std::sqrt(dx * dx + dy * dy);
    }

    /* Return a DescriptorLink with the given id
     */
    inline lama_msgs::DescriptorLink segmentDescriptorLink(const int32_t id)
    {
      lama_msgs::DescriptorLink descriptor_link;
      descriptor_link.descriptor_id = id;
      descriptor_link.interface_name = segment_interface_name_;
      return descriptor_link;
    }

    void callback_image(const sensor_msgs::ImageConstPtr& msg);
    void callback_odom(const nav_msgs::OdometryConstPtr& msg);

    // Publisher and subscribers.
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_handler_;
    ros::Subscriber odom_handler_;

    // Client proxies.
    ros::ServiceClient segment_setter_proxy_;

    // ROS parameters, shown outside.
    double matcher_max_relative_distance_;  //!> A potential descriptor is visible in the new image if
                                            //!> the distance to the best-match descriptor is smaller than
                                            //!> the distance to the second best match multiplied by this factor.
    double min_landmark_dist_;  //!> A landmark is saved if it was visible on at least such a traveled distance.
    double max_segment_length_;  //!> The jockey will successfully end the action if the traveled distance is at least this.
                                 //!> Defaults to 0, which means that the START action never finishes.

    // Hard-coded parameters.
    static const ros::Duration max_odom_age_;
    static const ros::Duration max_landmark_age_;

    // Internals.
    std::string segment_interface_name_;  //!> Name of the map interface for segments.
    std::string segment_setter_name_;  //!> Name of the service to write segments into the map.
    feature_extractor_function_ptr extract_features_;
    descriptor_matcher_function_ptr match_descriptors_;
    nav_msgs::Odometry odom_;  //!> Last received odometry message.
    bool has_odom_;
    bool image_processing_running_;  //!> true when treating an image.
    geometry_msgs::Pose start_pose_;  //!> Pose when learning started.
    Segment segment_;
    std::vector<PotentialLandmark> landmarks_; //!> Potential landmarks (features seen once).
};

} // namespace featurenav_base

#endif // FEATURENAV_BASE_AJOCKEY_H

