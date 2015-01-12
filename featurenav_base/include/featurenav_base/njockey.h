#ifndef FEATURENAV_BASE_NJOCKEY_H
#define FEATURENAV_BASE_NJOCKEY_H

#include <algorithm>  // std::for_each
#include <cmath>  // std::abs, std::max, std::min, std::ceil
#include <utility>  // std::pair

#include <boost/smart_ptr.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <lama_jockeys/navigating_jockey.h>

#include <featurenav_base/typedef.h>
#include <featurenav_base/GetSegment.h>
#include <featurenav_base/Landmark.h>

namespace featurenav_base {

using boost::accumulators::tag::density;

// Histogram with pixel distance as x-axis and density as y-axis.
typedef boost::iterator_range<vector<std::pair<double, double> >::iterator> histogram_type;
typedef boost::accumulators::accumulator_set<double, boost::accumulators::stats<density> > accumulator_type;

class NJockey : public lama_jockeys::NavigatingJockey
{
  public:

    NJockey(const std::string& name, const std::string& segment_interface_name, const std::string& segment_getter_name);

    void setExtractFeaturesFunction(feature_extractor_function_ptr f) {extract_features_ = f;}
    void setDescriptorMatcherFunction(descriptor_matcher_function_ptr f) {match_descriptors_ = f;}

    can_do_function_ptr canDo;
    start_do_function_ptr startDo;

  private:

    void reset();
    bool retrieveSegment();

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

    void callback_image(const sensor_msgs::ImageConstPtr& msg);
    void callback_odom(const nav_msgs::OdometryConstPtr& msg);

    geometry_msgs::Twist turnToAngle(const double direction);
    double saturate(double w) const;
    size_t processImage(const sensor_msgs::ImageConstPtr& image, double& w);
    vector<Landmark> select_tracked_landmarks(const double d) const;
    vector<double> compute_horizontal_differences(const vector<Landmark>& landmarks,
        const vector<cv::KeyPoint>& keypoints, const vector<Feature>& descriptors, const double d);

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

    // Subscribers and publishers.
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_handler_;
    ros::Subscriber odom_handler_;
    ros::Publisher twist_publisher_;

    // Client proxies.
    ros::ServiceClient segment_getter_proxy_;

    // ROS parameters (shown outside).
    double forward_velocity_;  //!> (m/s)
    double kp_;  //!> Proportional factor for the angular velocity (s^-1).
    double matcher_max_relative_distance_;  //!> A potential descriptor is visible in the new image if
                                            //!> the distance to the best-match descriptor is smaller than
                                            //!> the distance to the second best match multiplied by this factor.
    double max_angular_velocity_;  //!> (rad/s)
    double min_angular_velocity_;  //!> (rad/s)

    // Hard-coded parameters.
    static const ros::Duration max_odom_age_;
    static const int histogram_bin_size_;  //!> Histogram bin size in pixels.
    const static double reach_angular_distance_;  //!> dtheta to reach when turning (rad).

    // Internals.
    std::string segment_interface_name_;  //!> Name of the map interface for segments.
    std::string segment_getter_name_;  //!> Name of the service to write segments into the map.
    feature_extractor_function_ptr extract_features_;
    descriptor_matcher_function_ptr match_descriptors_;
    nav_msgs::Odometry odom_;  //!> Last received odometry message.
    bool has_odom_;  //!> true after an Odometry message was received.
    bool start_angle_reached_;  //!> true after the initial rotation.
    bool image_processing_running_;  //!> true when treating an image.
    geometry_msgs::Pose start_pose_;  //!> Pose when learning started.
    ::featurenav_base::Segment segment_;  //!> Segment we will traverse.
};

} // namespace featurenav_base

#endif // FEATURENAV_BASE_NJOCKEY_H

