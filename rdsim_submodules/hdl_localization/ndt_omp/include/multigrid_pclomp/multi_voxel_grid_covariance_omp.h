


















































#ifndef PCL_MULTI_VOXEL_GRID_COVARIANCE_OMP_H_
#define PCL_MULTI_VOXEL_GRID_COVARIANCE_OMP_H_

#include <pcl/pcl_macros.h>
#include <pcl/filters/boost.h>
#include <pcl/filters/voxel_grid.h>
#include <map>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
namespace pclomp
{
  






  template<typename PointT>
  class MultiVoxelGridCovariance : public pcl::VoxelGrid<PointT>
  {
    protected:
      using pcl::VoxelGrid<PointT>::filter_name_;
      using pcl::VoxelGrid<PointT>::getClassName;
      using pcl::VoxelGrid<PointT>::input_;
      using pcl::VoxelGrid<PointT>::indices_;
      using pcl::VoxelGrid<PointT>::filter_limit_negative_;
      using pcl::VoxelGrid<PointT>::filter_limit_min_;
      using pcl::VoxelGrid<PointT>::filter_limit_max_;

      
      using pcl::VoxelGrid<PointT>::leaf_size_;
      using pcl::VoxelGrid<PointT>::min_b_;
      using pcl::VoxelGrid<PointT>::max_b_;
      using pcl::VoxelGrid<PointT>::inverse_leaf_size_;
      using pcl::VoxelGrid<PointT>::div_b_;
      using pcl::VoxelGrid<PointT>::divb_mul_;

      typedef typename pcl::traits::fieldList<PointT>::type FieldList;
      typedef typename pcl::Filter<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
      typedef pcl::shared_ptr< pcl::VoxelGrid<PointT> > Ptr;
      typedef pcl::shared_ptr< const pcl::VoxelGrid<PointT> > ConstPtr;
#else
      typedef boost::shared_ptr< pcl::VoxelGrid<PointT> > Ptr;
      typedef boost::shared_ptr< const pcl::VoxelGrid<PointT> > ConstPtr;
#endif

      

      struct Leaf
      {
        


        Leaf () :
          nr_points (0),
          mean_ (Eigen::Vector3d::Zero ()),
          centroid (),
          cov_ (Eigen::Matrix3d::Identity ()),
          icov_ (Eigen::Matrix3d::Zero ()),
          evecs_ (Eigen::Matrix3d::Identity ()),
          evals_ (Eigen::Vector3d::Zero ())
        {
        }

        


        Eigen::Matrix3d
        getCov () const
        {
          return (cov_);
        }

        


        Eigen::Matrix3d
        getInverseCov () const
        {
          return (icov_);
        }

        


        Eigen::Vector3d
        getMean () const
        {
          return (mean_);
        }

        



        Eigen::Matrix3d
        getEvecs () const
        {
          return (evecs_);
        }

        



        Eigen::Vector3d
        getEvals () const
        {
          return (evals_);
        }

        


        int
        getPointCount () const
        {
          return (nr_points);
        }

        
        int nr_points;

        
        Eigen::Vector3d mean_;

        


        Eigen::VectorXf centroid;

        
        Eigen::Matrix3d cov_;

        
        Eigen::Matrix3d icov_;

        
        Eigen::Matrix3d evecs_;

        
        Eigen::Vector3d evals_;

      };

      struct LeafID {
        std::string parent_grid_id;
        int leaf_index;
        bool operator < (const LeafID& rhs) const {
          if (parent_grid_id < rhs.parent_grid_id) {
            return true;
          }
          if (parent_grid_id > rhs.parent_grid_id) {
            return false;
          }
          if (leaf_index < rhs.leaf_index) {
            return true;
          }
          if (leaf_index > rhs.leaf_index) {
            return false;
          }
          return false;
        }
      };

      
      typedef Leaf* LeafPtr;

      
      typedef const Leaf* LeafConstPtr;

      typedef std::map<LeafID, Leaf> LeafDict;

      struct BoundingBox
      {
        Eigen::Vector4i max;
        Eigen::Vector4i min;
        Eigen::Vector4i div_mul;
      };

    public:

      


      MultiVoxelGridCovariance () :
        min_points_per_voxel_ (6),
        min_covar_eigvalue_mult_ (0.01),
        leaves_ (),
        grid_leaves_ (),
        leaf_indices_ (),
        kdtree_ ()
      {
        leaf_size_.setZero ();
        min_b_.setZero ();
        max_b_.setZero ();
        filter_name_ = "MultiVoxelGridCovariance";
      }

      

      inline void
      setInputCloudAndFilter (const PointCloudConstPtr &cloud, const std::string &grid_id)
      {
        LeafDict leaves;
        applyFilter (cloud, grid_id, leaves);

        grid_leaves_[grid_id] = leaves;
      }

      inline void 
      removeCloud (const std::string &grid_id)
      {
        grid_leaves_.erase(grid_id);
      }

      inline void 
      createKdtree ()
      {
        leaves_.clear();
        for (const auto &kv: grid_leaves_)
        {
          leaves_.insert(kv.second.begin(), kv.second.end());
        }

        leaf_indices_.clear();
        voxel_centroids_ptr_.reset(new PointCloud);
        voxel_centroids_ptr_->height = 1;
        voxel_centroids_ptr_->is_dense = true;
        voxel_centroids_ptr_->points.clear();
        voxel_centroids_ptr_->points.reserve(leaves_.size ());
        for (const auto & element: leaves_)
        {
          leaf_indices_.push_back(element.first);
          voxel_centroids_ptr_->push_back (PointT ());
          voxel_centroids_ptr_->points.back ().x = element.second.centroid[0];
          voxel_centroids_ptr_->points.back ().y = element.second.centroid[1];
          voxel_centroids_ptr_->points.back ().z = element.second.centroid[2];
        }
        voxel_centroids_ptr_->width = static_cast<uint32_t> (voxel_centroids_ptr_->points.size ());

        if (voxel_centroids_ptr_->size() > 0)
        {
          kdtree_.setInputCloud (voxel_centroids_ptr_);
        }
      }

      








      int
      radiusSearch (const PointT &point, double radius, std::vector<LeafConstPtr> &k_leaves,
                    std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
      {
        k_leaves.clear ();

        
        std::vector<int> k_indices;
        int k = kdtree_.radiusSearch (point, radius, k_indices, k_sqr_distances, max_nn);

        
        k_leaves.reserve (k);
        for (std::vector<int>::iterator iter = k_indices.begin (); iter != k_indices.end (); iter++)
        {
          auto leaf = leaves_.find(leaf_indices_[*iter]);
          if (leaf == leaves_.end()) {
            std::cerr << "error : could not find the leaf corresponding to the voxel" << std::endl;
            std::cin.ignore(1);
          }
          k_leaves.push_back (&(leaf->second));
        }
        return k;
      }

      









      inline int
      radiusSearch (const PointCloud &cloud, int index, double radius,
                    std::vector<LeafConstPtr> &k_leaves, std::vector<float> &k_sqr_distances,
                    unsigned int max_nn = 0) const
      {
        if (index >= static_cast<int> (cloud.points.size ()) || index < 0)
          return (0);
        return (radiusSearch (cloud.points[index], radius, k_leaves, k_sqr_distances, max_nn));
      }

      PointCloud getVoxelPCD () const
      {
        return *voxel_centroids_ptr_;
      }

  		std::vector<std::string> getCurrentMapIDs() const
      {
        std::vector<std::string> output{};
        for (const auto &element: grid_leaves_) {
          output.push_back(element.first);
        }
        return output;
      }

    protected:

      


      void applyFilter (const PointCloudConstPtr &input, const std::string &grid_id, LeafDict &leaves) const;

      void updateVoxelCentroids (const Leaf &leaf, PointCloud &voxel_centroids) const;

      void updateLeaf (const PointT &point, const int &centroid_size, Leaf &leaf) const;

      void computeLeafParams (const Eigen::Vector3d &pt_sum,
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> &eigensolver,
        Leaf &leaf) const;

      LeafID getLeafID (const std::string &grid_id, const PointT &point, const BoundingBox &bbox) const;

      
      int min_points_per_voxel_;

      
      double min_covar_eigvalue_mult_;

      
	    LeafDict leaves_;

      
      std::map<std::string, LeafDict> grid_leaves_;

      
      std::vector<LeafID> leaf_indices_;

      
      pcl::KdTreeFLANN<PointT> kdtree_;

      PointCloudPtr voxel_centroids_ptr_;
  };
}

#endif  
