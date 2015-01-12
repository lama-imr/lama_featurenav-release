#include <anj_featurenav/jockey.h>

#define GET_PARAM(prefix, type_t, name, default) \
  type_t name = default; \
  private_nh_.getParam(#prefix"/"#name, name); \
  ROS_DEBUG_STREAM(#prefix"/"#name": " << name);

#define GET_PARAM_DETECTOR(type_t, name, default) GET_PARAM(feature_detector, type_t, name, default)
#define GET_PARAM_EXTRACTOR(type_t, name, default) GET_PARAM(descriptor_extractor, type_t, name, default)
#define GET_PARAM_MATCHER(type_t, name, default) GET_PARAM(descriptor_matcher, type_t, name, default)

namespace anj_featurenav
{

Jockey::Jockey(const std::string& name) :
  feature_detector_type_("FAST"),
  descriptor_extractor_type_("BRIEF"),
  /* descriptor_matcher_type_("FlannBased"), */
  descriptor_matcher_type_("BruteForce"),
  private_nh_("~"),
  base_name_(name)
{
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Initialize the feature detector.
  private_nh_.getParam("feature_detector/type", feature_detector_type_);
  ROS_DEBUG_STREAM("Feature detector type: " << feature_detector_type_);

  if (feature_detector_type_ == "FAST")
  {
    initDetectorFast();
  }
  else if (feature_detector_type_ == "STAR")
  {
    initDetectorStar();
  }
  else if (feature_detector_type_ == "SIFT")
  {
    throw ros::InvalidParameterException("SIFT algorithm is nonfree, cf. anj_featurenav_nonfree package");
  }
  else if (feature_detector_type_ == "SURF")
  {
    throw ros::InvalidParameterException("SURF algorithm is nonfree, cf. anj_featurenav_nonfree package");
  }
  else if (feature_detector_type_ == "ORB")
  {
    initDetectorOrb();
  }
  else if (feature_detector_type_ == "MSER")
  {
    initDetectorMser();
  }
  else if (feature_detector_type_ == "GFTT")
  {
    initDetectorGftt();
  }
  else if (feature_detector_type_ == "Dense")
  {
    initDetectorDense();
  }
  else if (feature_detector_type_ == "SimpleBlob")
  {
    initDetectorSimpleblob();
  }
  else
  {
    throw ros::InvalidParameterException(std::string("Unkown feature detection algorithm (") +
        feature_detector_type_ + std::string(")"));
  }

  // Initialize the descriptor (feature) extractor.
  private_nh_.getParam("descriptor_extractor/type", descriptor_extractor_type_);
  ROS_DEBUG_STREAM("Descriptor extractor type: " << descriptor_extractor_type_);

  if (descriptor_extractor_type_ == "SIFT")
  {
    throw ros::InvalidParameterException("SIFT algorithm is nonfree, cf. anj_featurenav_nonfree package");
  }
  else if (descriptor_extractor_type_ == "SURF")
  {
    throw ros::InvalidParameterException("SURF algorithm is nonfree, cf. anj_featurenav_nonfree package");
  }
  else if (descriptor_extractor_type_ == "ORB")
  {
    initExtractorOrb();
  }
  else if (descriptor_extractor_type_ == "BRIEF")
  {
    initExtractorBrief();
  }
  else
  {
    throw ros::InvalidParameterException(std::string("Unkown descriptor extraction algorithm (") +
        descriptor_extractor_type_ + std::string(")"));
  }

  // Initialize the descriptor matcher.
  private_nh_.getParam("descriptor_matcher/type", descriptor_matcher_type_);
  ROS_DEBUG_STREAM("Descriptor matcher type: " << descriptor_matcher_type_);

  if (descriptor_matcher_type_ == "BruteForce")
  {
    initMatcherBruteforce();
  }
  else if (descriptor_matcher_type_ == "FlannBased")
  {
    initMatcherFlannbased();
  }
  else
  {
    throw ros::InvalidParameterException(std::string("Unkown descriptor matcher algorithm (") +
        descriptor_matcher_type_ + std::string(")"));
  }
  
  map_interface_name_ = base_name_ + "_" + feature_detector_code_ + "_" + descriptor_extractor_code_;
  anjockey_ptr_.reset(new featurenav_base::ANJockey(base_name_, map_interface_name_));
  anjockey_ptr_->setExtractFeaturesFunction(boost::bind(&Jockey::extractFeatures, this, _1, _2, _3));
  anjockey_ptr_->setDescriptorMatcherFunction(boost::bind(&Jockey::matchDescriptors, this, _1, _2, _3));
}

std::string Jockey::getLearningJockeyName() const
{
  if (anjockey_ptr_ == NULL)
  {
    return "";
  }
  return anjockey_ptr_->getLearningJockeyName();
}

std::string Jockey::getNavigatingJockeyName() const
{
  if (anjockey_ptr_ == NULL)
  {
    return "";
  }
  return anjockey_ptr_->getNavigatingJockeyName();
}

/* Initialize feature detector with FAST algorithm
*/
void Jockey::initDetectorFast()
{
  // Initialize feature detector.
  GET_PARAM_DETECTOR(int, threshold, 1);
  GET_PARAM_DETECTOR(bool, nonmax_suppression, true);

  feature_detector_.reset(new cv::FastFeatureDetector(threshold, nonmax_suppression));
  feature_detector_code_ = "fast";
}

/* Initialize feature detector with STAR algorithm
*/
void Jockey::initDetectorStar()
{
  GET_PARAM_DETECTOR(int, max_size, 16);
  GET_PARAM_DETECTOR(int, response_threshold, 30);
  GET_PARAM_DETECTOR(int, line_threshold_projected, 10);
  GET_PARAM_DETECTOR(int, line_threshold_binarized, 8);
  GET_PARAM_DETECTOR(int, suppress_nonmax_size, 5);

  feature_detector_.reset(new cv::StarFeatureDetector(max_size, response_threshold, line_threshold_projected,
        line_threshold_binarized, suppress_nonmax_size));
  feature_detector_code_ = "star";
}

/* Initialize feature detector with Orb algorithm
*/
void Jockey::initDetectorOrb()
{
  enum
  {
    HARRIS_SCORE = 0,
    FAST_SCORE = 1
  };

  GET_PARAM_DETECTOR(double, scale_factor, 1.2);
  GET_PARAM_DETECTOR(int, n_features, 500);
  GET_PARAM_DETECTOR(int, n_levels, 8);
  GET_PARAM_DETECTOR(int, edge_threshold, 31);
  GET_PARAM_DETECTOR(int, first_level, 0);
  GET_PARAM_DETECTOR(int, wta_k, 2);
  GET_PARAM_DETECTOR(int, score_type, HARRIS_SCORE);
  GET_PARAM_DETECTOR(int, patch_size, 31);

  feature_detector_.reset(new cv::OrbFeatureDetector(n_features, scale_factor, n_levels, edge_threshold,
        first_level, wta_k, score_type, patch_size));
  feature_detector_code_ = "orb";
}

/* Initialize feature detector with MSER algorithm
*/
void Jockey::initDetectorMser()
{
  GET_PARAM_DETECTOR(int, delta, 5);
  GET_PARAM_DETECTOR(int, min_area, 60);
  GET_PARAM_DETECTOR(int, max_area, 14400);
  GET_PARAM_DETECTOR(double, max_variation, 0.25);
  GET_PARAM_DETECTOR(double, min_diversity, 0.2);
  GET_PARAM_DETECTOR(int, max_evolution, 200);
  GET_PARAM_DETECTOR(double, area_threshold, 1.01);
  GET_PARAM_DETECTOR(double, min_margin, 0.003);
  GET_PARAM_DETECTOR(int, edge_blur_size, 5);

  feature_detector_.reset(new cv::MserFeatureDetector(delta, min_area, max_area, max_variation, min_diversity, max_evolution, area_threshold, min_margin,
        edge_blur_size));
  feature_detector_code_ = "mser";
}

/* Initialize feature detector with GFTT algorithm (Good Features To Track)
*/
void Jockey::initDetectorGftt()
{
  GET_PARAM_DETECTOR(int, max_corners, 1000);
  GET_PARAM_DETECTOR(int, block_size, 3);
  GET_PARAM_DETECTOR(double, quality_level, 0.01);
  GET_PARAM_DETECTOR(double, min_distance, 1);
  GET_PARAM_DETECTOR(double, k, 0.04);
  GET_PARAM_DETECTOR(bool, use_harris_detector, false);

  feature_detector_.reset(new cv::GoodFeaturesToTrackDetector(max_corners, quality_level, min_distance, block_size, use_harris_detector, k));
  feature_detector_code_ = "gfft";
}

/* Initialize feature detector with Dense algorithm
*/
void Jockey::initDetectorDense()
{
  GET_PARAM_DETECTOR(int, feature_scale_levels, 1);
  GET_PARAM_DETECTOR(int, init_xy_step, 6);
  GET_PARAM_DETECTOR(int, init_img_bound, 0);
  GET_PARAM_DETECTOR(double, init_feature_scale, 1);
  GET_PARAM_DETECTOR(double, feature_scale_mul, 0.1);
  GET_PARAM_DETECTOR(bool, vary_xy_step_with_scale, true);
  GET_PARAM_DETECTOR(bool, vary_img_bound_with_scale, false);

  feature_detector_.reset(new cv::DenseFeatureDetector(init_feature_scale, feature_scale_levels, feature_scale_mul,
        init_xy_step, init_img_bound, vary_xy_step_with_scale, vary_img_bound_with_scale));
  feature_detector_code_ = "dense";
}

/* Initialize feature detector with SimpleBlob algorithm
*/
void Jockey::initDetectorSimpleblob()
{
  feature_detector_.reset(new cv::SimpleBlobDetector);
  feature_detector_code_ = "blob";
}

/* Initialize feature extractor with ORB algorithm
*/
void Jockey::initExtractorOrb()
{
  enum
  {
    HARRIS_SCORE = 0,
    FAST_SCORE = 1
  };

  GET_PARAM_EXTRACTOR(double, scale_factor, 1.2);
  GET_PARAM_EXTRACTOR(int, n_features, 500);
  GET_PARAM_EXTRACTOR(int, n_levels, 8);
  GET_PARAM_EXTRACTOR(int, edge_threshold, 31);
  GET_PARAM_EXTRACTOR(int, first_level, 0);
  GET_PARAM_EXTRACTOR(int, wta_k, 2);
  GET_PARAM_EXTRACTOR(int, score_type, HARRIS_SCORE);
  GET_PARAM_EXTRACTOR(int, patch_size, 31);

  descriptor_extractor_.reset(new cv::OrbDescriptorExtractor(n_features, scale_factor,
      n_levels, edge_threshold, first_level, wta_k, score_type, patch_size));
  descriptor_extractor_code_ = "orb";
}

/* Initialize feature extractor with Brief algorithm
*/
void Jockey::initExtractorBrief()
{
  GET_PARAM_EXTRACTOR(int, bytes, 32);

  descriptor_extractor_.reset(new cv::BriefDescriptorExtractor(bytes));
  descriptor_extractor_code_ = "brief";
}

void Jockey::initMatcherBruteforce()
{
  int norm_type = cv::NORM_L1;

  GET_PARAM_MATCHER(std::string, norm, "NORM_L2");
  GET_PARAM_MATCHER(bool, cross_check, false);

  if ((norm == "NORM_L1") || (norm == "L1"))
  {
    norm_type = cv::NORM_L1;
  }
  else if ((norm == "NORM_L2") || (norm == "L2"))
  {
    norm_type = cv::NORM_L2;
  }
  else if ((norm == "NORM_HAMMING") || (norm == "HAMMING"))
  {
    norm_type = cv::NORM_HAMMING;
  }
  else if ((norm == "NORM_HAMMING2") || (norm == "HAMMING2"))
  {
    norm_type = cv::NORM_HAMMING2;
  }
  else
  {
    ROS_WARN_STREAM(ros::this_node::getName() << ": unknown norm type, defaulting to L2");
  }

  descriptor_matcher_.reset(new cv::BFMatcher(norm_type, cross_check));
  descriptor_matcher_code_ = "bf";
}

void Jockey::initMatcherFlannbased()
{
  descriptor_matcher_.reset(new cv::FlannBasedMatcher());
  descriptor_matcher_code_ = "flann";
  /* descriptor_matcher_.reset(cv::DescriptorMatcher::create("FlannBased")); */
}

void Jockey::extractFeatures(const sensor_msgs::ImageConstPtr& image, vector<KeyPoint>& keypoints, vector<Feature>& descriptors) const
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s, doing nothing...", e.what());
    return;
  }

  feature_detector_->detect(cv_ptr->image, keypoints);
  cv::Mat cv_descriptors;
  descriptor_extractor_->compute(cv_ptr->image, keypoints, cv_descriptors);

  rosFromCv(cv_descriptors, descriptors);
}

void Jockey::matchDescriptors(const vector<Feature>& query_descriptors, const vector<Feature>& train_descriptors, vector<vector<DMatch> >& matches) const
{
  int type = -1;
  if ((descriptor_extractor_code_ == "orb") || (descriptor_extractor_code_ == "brief"))
  {
    type = CV_8U;
  }

  cv::Mat cv_query_descriptors;
  cvFromRos(query_descriptors, cv_query_descriptors, type);

  cv::Mat cv_train_descriptors;
  cvFromRos(train_descriptors, cv_train_descriptors, type);

  // FlannBasedMatcher requires CV_32F descriptor type.
  if (descriptor_matcher_code_ == "flann")
  {
    cv::Mat cv_query_descriptors_copy;
    if(cv_query_descriptors.type() != CV_32F)
    {
      cv_query_descriptors.convertTo(cv_query_descriptors_copy, CV_32F);
    }
    else
    {
      cv_query_descriptors_copy = cv_query_descriptors;
    }
    cv::Mat cv_train_descriptors_copy;
    if(cv_train_descriptors.type() != CV_32F)
    {
      cv_train_descriptors.convertTo(cv_train_descriptors_copy, CV_32F);
    }
    else
    {
      cv_train_descriptors_copy = cv_train_descriptors;
    }
    descriptor_matcher_->knnMatch(cv_query_descriptors_copy, cv_train_descriptors_copy, matches, 2);
  }
  else
  {
    descriptor_matcher_->knnMatch(cv_query_descriptors, cv_train_descriptors, matches, 2);
  }
}

} /* anj_featurenav */ 

