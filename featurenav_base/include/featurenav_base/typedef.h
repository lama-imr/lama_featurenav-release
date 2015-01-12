#ifndef FEATURENAV_BASE_TYPEDEF_H
#define FEATURENAV_BASE_TYPEDEF_H

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <sensor_msgs/Image.h>

#include <featurenav_base/Feature.h>

namespace featurenav_base {

using std::vector;
using cv::KeyPoint;
using cv::DMatch;
using ::featurenav_base::Feature;

// void(image[in], keypoints[out], descriptors[out])
typedef boost::function<void(const sensor_msgs::ImageConstPtr&, vector<KeyPoint>&, vector<Feature>&)> feature_extractor_function_ptr;
// void(query_descriptors[in], train_descriptors[in], matches[out])
typedef boost::function<void(const vector<Feature>&, const vector<Feature>&, vector<vector<DMatch> >&)> descriptor_matcher_function_ptr;

enum action_type {LEARN, NAVIGATE};

typedef boost::function<bool(const action_type)> can_do_function_ptr;
typedef boost::function<void(const action_type)> start_do_function_ptr;

} // namespace featurenav_base

#endif // FEATURENAV_BASE_TYPEDEF_H
