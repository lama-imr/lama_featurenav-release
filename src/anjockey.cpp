#include <featurenav_base/anjockey.h>

namespace featurenav_base {

ANJockey::ANJockey(const std::string& name, const std::string& segment_interface_name) :
  nh_(),
  private_nh_("~"),
  segment_interface_name_(segment_interface_name),
  learning_(false),
  navigating_(false)
{
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  if (!initMapSegmentInterface())
  {
    throw ros::Exception("Initialization error");
  }

  ajockey_ptr_.reset(new AJockey(name + "_learner", segment_interface_name_, segment_setter_name_));
  // TODO: add segment_getter_name_ parameter.
  njockey_ptr_.reset(new NJockey(name + "_navigator", segment_interface_name_, segment_getter_name_));

  // Initialize the clients for the LaserScan getter and setter services (interface to map).
  ajockey_ptr_->canDo = (can_do_function_ptr) boost::bind(&ANJockey::canDo, this, _1);
  njockey_ptr_->canDo = (can_do_function_ptr) boost::bind(&ANJockey::canDo, this, _1);
  ajockey_ptr_->startDo = (start_do_function_ptr) boost::bind(&ANJockey::startDo, this, _1);
  njockey_ptr_->startDo = (start_do_function_ptr) boost::bind(&ANJockey::startDo, this, _1);
}

/* Create the getter and setter services for Segment messages.
 */
bool ANJockey::initMapSegmentInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  ROS_DEBUG_STREAM(ros::this_node::getName() << ": waiting for service /interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = segment_interface_name_;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::SERIALIZED;
  srv.request.get_service_message = "featurenav_base/GetSegment";
  srv.request.set_service_message = "featurenav_base/SetSegment";
  if (!client.call(srv))
  {
    ROS_ERROR_STREAM("Failed to create the Lama interface " << segment_interface_name_);
    return false;
  }
  segment_getter_name_ = srv.response.get_service_name;
  segment_setter_name_ = srv.response.set_service_name;
  return true;
}

std::string ANJockey::getLearningJockeyName() const
{
  if (ajockey_ptr_ == NULL)
  {
    return "";
  }
  return ajockey_ptr_->getName();
}

std::string ANJockey::getNavigatingJockeyName() const
{
  if (njockey_ptr_ == NULL)
  {
    return "";
  }
  return njockey_ptr_->getName();
}

void ANJockey::setExtractFeaturesFunction(feature_extractor_function_ptr f)
{
  ajockey_ptr_->setExtractFeaturesFunction(f);
  njockey_ptr_->setExtractFeaturesFunction(f);
}

void ANJockey::setDescriptorMatcherFunction(descriptor_matcher_function_ptr f)
{
  ajockey_ptr_->setDescriptorMatcherFunction(f);
  njockey_ptr_->setDescriptorMatcherFunction(f);
}

bool ANJockey::canDo(const action_type action)
{
  if (learning_ && (action == NAVIGATE))
    return false;
  if (navigating_ && (action == LEARN))
    return false;
  return true;
}

void ANJockey::startDo(const action_type action)
{
  learning_ = (action == LEARN);
  navigating_ = (action == NAVIGATE);
}

} // namespace featurenav_base


