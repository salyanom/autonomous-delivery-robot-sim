









































#ifndef HDL_GLOBAL_LOCALIZATION_RANSAC_POSE_ESTIMATION_HPP
#define HDL_GLOBAL_LOCALIZATION_RANSAC_POSE_ESTIMATION_HPP

#include <atomic>
#include <vector>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <hdl_global_localization/global_localization_results.hpp>
#include <hdl_global_localization/ransac/voxelset.hpp>
#include <hdl_global_localization/ransac/matching_cost_evaluater.hpp>

namespace hdl_global_localization {

typedef struct {
  bool voxel_based;
  double max_correspondence_distance;
  double similarity_threshold;
  int correspondence_randomness;
  int max_iterations;
  int matching_budget;
  double inlier_fraction;
} RansacPoseParams;

template <typename FeatureT>
class RansacPoseEstimation {
public:
  RansacPoseEstimation(rclcpp::Node::SharedPtr node);

  void set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, typename pcl::PointCloud<FeatureT>::ConstPtr target_features);
  void set_source(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, typename pcl::PointCloud<FeatureT>::ConstPtr source_features);

  GlobalLocalizationResults estimate();
  rcl_interfaces::msg::SetParametersResult parameter_update(const std::vector<rclcpp::Parameter> & parameters);

private:
  void select_samples(std::mt19937& mt, const std::vector<std::vector<int>>& similar_features, std::vector<int>& samples, std::vector<int>& correspondences) const;


private:
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target;
  typename pcl::PointCloud<FeatureT>::ConstPtr target_features;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr source;
  typename pcl::PointCloud<FeatureT>::ConstPtr source_features;

  typename pcl::KdTreeFLANN<FeatureT>::Ptr feature_tree;
  std::unique_ptr<MatchingCostEvaluater> evaluater;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  RansacPoseParams params;
  rclcpp::Node::SharedPtr node;
};

}  

#endif