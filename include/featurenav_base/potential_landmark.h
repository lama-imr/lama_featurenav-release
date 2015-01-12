#ifndef FEATURENAV_BASE_POTENTIAL_LANDMARK_H
#define FEATURENAV_BASE_POTENTIAL_LANDMARK_H

#include <ros/ros.h>

#include <featurenav_base/Feature.h>
#include <featurenav_base/Landmark.h>

namespace featurenav_base
{

/* A struct containng a ::featurenav_base::Landmark with extra information
 */
struct PotentialLandmark
{
  PotentialLandmark(const Feature& descriptor, const ros::Time time, const double x, const double y, const double d) :
    landmark(),
    last_seen(time)
  {
    landmark.descriptor = descriptor;
    landmark.u.x = x;
    landmark.u.y = y;
    landmark.v = landmark.u;
    landmark.du = d;
    landmark.dv = d;
  }

  Landmark landmark;
  ros::Time last_seen;
};
  
} /* namespace featurenav_base */ 

#endif // FEATURENAV_BASE_POTENTIAL_LANDMARK_H
