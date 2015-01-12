#ifndef FEATURENAV_BASE_ANJOCKEY_H
#define FEATURENAV_BASE_ANJOCKEY_H

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <lama_interfaces/AddInterface.h>

#include <featurenav_base/typedef.h>
#include <featurenav_base/ajockey.h>
#include <featurenav_base/njockey.h>

namespace featurenav_base {

class ANJockey
{
  public:

    ANJockey(const std::string& name, const std::string& segment_interface_name);

    std::string getLearningJockeyName() const;
    std::string getNavigatingJockeyName() const;

    void setExtractFeaturesFunction(feature_extractor_function_ptr f);
    void setDescriptorMatcherFunction(descriptor_matcher_function_ptr f);

  private:

    bool initMapSegmentInterface();
    bool canDo(const action_type action);
    void startDo(const action_type action);

    // Internals.
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string segment_interface_name_;  //!> Name of the map interface for segments.
    std::string segment_getter_name_;  //!> Name of the service to retrieve segments from the map.
    std::string segment_setter_name_;  //!> Name of the service to write segments into the map.

    boost::scoped_ptr<AJockey> ajockey_ptr_;
    boost::scoped_ptr<NJockey> njockey_ptr_;

    bool learning_;
    bool navigating_;
};

} // namespace featurenav_base

#endif /* FEATURENAV_BASE_ANJOCKEY_H */
