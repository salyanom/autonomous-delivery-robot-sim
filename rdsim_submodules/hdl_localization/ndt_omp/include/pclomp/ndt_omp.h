







































#ifndef PCL_REGISTRATION_NDT_OMP_H_
#define PCL_REGISTRATION_NDT_OMP_H_

#include "boost/optional.hpp"

#include <pcl/registration/registration.h>
#include <pcl/search/impl/search.hpp>
#include "voxel_grid_covariance_omp.h"

#include <unsupported/Eigen/NonLinearOptimization>

namespace pclomp
{
	enum NeighborSearchMethod {
		KDTREE,
		DIRECT26,
		DIRECT7,
		DIRECT1
	};

	struct NdtResult
	{
		Eigen::Matrix4f pose;
		float transform_probability;
		float nearest_voxel_transformation_likelihood;
		int iteration_num;
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transformation_array;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct NdtParams
	{
		double trans_epsilon;
		double step_size;
		double resolution;
		int max_iterations;
		pclomp::NeighborSearchMethod search_method;
		int num_threads;
		float regularization_scale_factor;
	};

	










	template<typename PointSource, typename PointTarget>
	class NormalDistributionsTransform : public pcl::Registration<PointSource, PointTarget>
	{
	protected:

		typedef typename pcl::Registration<PointSource, PointTarget>::PointCloudSource PointCloudSource;
		typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
		typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

		typedef typename pcl::Registration<PointSource, PointTarget>::PointCloudTarget PointCloudTarget;
		typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
		typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

		typedef pcl::PointIndices::Ptr PointIndicesPtr;
		typedef pcl::PointIndices::ConstPtr PointIndicesConstPtr;

		
		typedef pclomp::VoxelGridCovariance<PointTarget> TargetGrid;
		
		typedef TargetGrid* TargetGridPtr;
		
		typedef const TargetGrid* TargetGridConstPtr;
		
		typedef typename TargetGrid::LeafConstPtr TargetGridLeafConstPtr;


	public:

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
		typedef pcl::shared_ptr< NormalDistributionsTransform<PointSource, PointTarget> > Ptr;
		typedef pcl::shared_ptr< const NormalDistributionsTransform<PointSource, PointTarget> > ConstPtr;
#else
		typedef boost::shared_ptr< NormalDistributionsTransform<PointSource, PointTarget> > Ptr;
		typedef boost::shared_ptr< const NormalDistributionsTransform<PointSource, PointTarget> > ConstPtr;
#endif


		


		NormalDistributionsTransform();

		
		virtual ~NormalDistributionsTransform() {}

		void setNumThreads(int n)
		{
			num_threads_ = n;
		}

		inline int getNumThreads() const
		{
			return num_threads_;
		}

		


		inline void
			setInputTarget(const PointCloudTargetConstPtr &cloud)
		{
			pcl::Registration<PointSource, PointTarget>::setInputTarget(cloud);
			init();
		}

		


		inline void
			setResolution(float resolution)
		{
			
			if (resolution_ != resolution)
			{
				resolution_ = resolution;
				if (input_)
					init();
			}
		}

		


		inline float
			getResolution() const
		{
			return (resolution_);
		}

		


		inline double
			getStepSize() const
		{
			return (step_size_);
		}

		


		inline void
			setStepSize(double step_size)
		{
			step_size_ = step_size;
		}

		


		inline double
			getOutlierRatio() const
		{
			return (outlier_ratio_);
		}

		


		inline void
			setOutlierRatio(double outlier_ratio)
		{
			outlier_ratio_ = outlier_ratio;
		}

		inline void setNeighborhoodSearchMethod(NeighborSearchMethod method) {
			search_method = method;
		}

		inline NeighborSearchMethod
			getNeighborhoodSearchMethod() const
		{
			return search_method;
		}

		


		inline double
			getTransformationProbability() const
		{
			return (trans_probability_);
		}

		inline double
			getNearestVoxelTransformationLikelihood() const
		{
			return nearest_voxel_transformation_likelihood_;
		}

		


		inline int
			getFinalNumIteration() const
		{
			return (nr_iterations_);
		}

		
		inline Eigen::Matrix<double, 6, 6>
			getHessian() const
		{
			return hessian_;
		}

		
		inline const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
			getFinalTransformationArray() const
		{
			return transformation_array_;
		}

		



		static void
			convertTransform(const Eigen::Matrix<double, 6, 1> &x, Eigen::Affine3f &trans)
		{
			trans = Eigen::Translation<float, 3>(float(x(0)), float(x(1)), float(x(2))) *
				Eigen::AngleAxis<float>(float(x(3)), Eigen::Vector3f::UnitX()) *
				Eigen::AngleAxis<float>(float(x(4)), Eigen::Vector3f::UnitY()) *
				Eigen::AngleAxis<float>(float(x(5)), Eigen::Vector3f::UnitZ());
		}

		



		static void
			convertTransform(const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix4f &trans)
		{
			Eigen::Affine3f _affine;
			convertTransform(x, _affine);
			trans = _affine.matrix();
		}

		
		
		double calculateScore(const PointCloudSource& cloud) const;
		double calculateTransformationProbability(const PointCloudSource& cloud) const;
		double calculateNearestVoxelTransformationLikelihood(const PointCloudSource& cloud) const;

		inline void setRegularizationScaleFactor(float regularization_scale_factor)
		{
			regularization_scale_factor_ = regularization_scale_factor;
		}

		inline void setRegularizationPose(Eigen::Matrix4f regularization_pose)
		{
			regularization_pose_ = regularization_pose;
		}

		inline void unsetRegularizationPose()
		{
			regularization_pose_ = boost::none;
		}

		NdtResult getResult()
		{
			NdtResult ndt_result;
			ndt_result.pose = this->getFinalTransformation();
			ndt_result.transformation_array = getFinalTransformationArray();
			ndt_result.transform_probability = getTransformationProbability();
			ndt_result.nearest_voxel_transformation_likelihood =
				getNearestVoxelTransformationLikelihood();
			ndt_result.iteration_num = getFinalNumIteration();
			return ndt_result;
		}

		void setParams(const NdtParams & ndt_params)
		{
			this->setTransformationEpsilon(ndt_params.trans_epsilon);
			this->setStepSize(ndt_params.step_size);
			this->setResolution(ndt_params.resolution);
			this->setMaximumIterations(ndt_params.max_iterations);
			setRegularizationScaleFactor(ndt_params.regularization_scale_factor);
			setNeighborhoodSearchMethod(ndt_params.search_method);
			setNumThreads(ndt_params.num_threads);
		}

		NdtParams getParams() const
		{
			NdtParams ndt_params;
			ndt_params.trans_epsilon = transformation_epsilon_;
			ndt_params.step_size = getStepSize();
			ndt_params.resolution = getResolution();
			ndt_params.max_iterations = max_iterations_;
			ndt_params.regularization_scale_factor = regularization_scale_factor_;
			ndt_params.search_method = getNeighborhoodSearchMethod();
			ndt_params.num_threads = num_threads_;
			return ndt_params;
		}

	protected:

		using pcl::Registration<PointSource, PointTarget>::reg_name_;
		using pcl::Registration<PointSource, PointTarget>::getClassName;
		using pcl::Registration<PointSource, PointTarget>::input_;
		using pcl::Registration<PointSource, PointTarget>::indices_;
		using pcl::Registration<PointSource, PointTarget>::target_;
		using pcl::Registration<PointSource, PointTarget>::nr_iterations_;
		using pcl::Registration<PointSource, PointTarget>::max_iterations_;
		using pcl::Registration<PointSource, PointTarget>::previous_transformation_;
		using pcl::Registration<PointSource, PointTarget>::final_transformation_;
		using pcl::Registration<PointSource, PointTarget>::transformation_;
		using pcl::Registration<PointSource, PointTarget>::transformation_epsilon_;
		using pcl::Registration<PointSource, PointTarget>::converged_;
		using pcl::Registration<PointSource, PointTarget>::corr_dist_threshold_;
		using pcl::Registration<PointSource, PointTarget>::inlier_threshold_;

		using pcl::Registration<PointSource, PointTarget>::update_visualizer_;

		


		virtual void
			computeTransformation(PointCloudSource &output)
		{
			computeTransformation(output, Eigen::Matrix4f::Identity());
		}

		



		virtual void
			computeTransformation(PointCloudSource &output, const Eigen::Matrix4f &guess);

		
		void inline
			init()
		{
			target_cells_.setLeafSize(resolution_, resolution_, resolution_);
			target_cells_.setInputCloud(target_);
			
			target_cells_.filter(true);
		}

		







		double
			computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient,
				Eigen::Matrix<double, 6, 6> &hessian,
				PointCloudSource &trans_cloud,
				Eigen::Matrix<double, 6, 1> &p,
				bool compute_hessian = true);

		







		double
			updateDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient,
				Eigen::Matrix<double, 6, 6> &hessian,
				const Eigen::Matrix<float, 4, 6> &point_gradient_,
				const Eigen::Matrix<float, 24, 6> &point_hessian_,
				const Eigen::Vector3d &x_trans, const Eigen::Matrix3d &c_inv,
				bool compute_hessian = true) const;

		




		void
			computeAngleDerivatives(Eigen::Matrix<double, 6, 1> &p, bool compute_hessian = true);

		




		void
			computePointDerivatives(Eigen::Vector3d &x, Eigen::Matrix<double, 3, 6>& point_gradient_, Eigen::Matrix<double, 18, 6>& point_hessian_, bool compute_hessian = true) const;

		void
			computePointDerivatives(Eigen::Vector3d &x, Eigen::Matrix<float, 4, 6>& point_gradient_, Eigen::Matrix<float, 24, 6>& point_hessian_, bool compute_hessian = true) const;

		





		void
			computeHessian(Eigen::Matrix<double, 6, 6> &hessian,
				PointCloudSource &trans_cloud,
				Eigen::Matrix<double, 6, 1> &p);

		





		void
			updateHessian(Eigen::Matrix<double, 6, 6> &hessian,
				const Eigen::Matrix<double, 3, 6> &point_gradient_,
				const Eigen::Matrix<double, 18, 6> &point_hessian_,
				const Eigen::Vector3d &x_trans, const Eigen::Matrix3d &c_inv) const;

		












		double
			computeStepLengthMT(const Eigen::Matrix<double, 6, 1> &x,
				Eigen::Matrix<double, 6, 1> &step_dir,
				double step_init,
				double step_max, double step_min,
				double &score,
				Eigen::Matrix<double, 6, 1> &score_gradient,
				Eigen::Matrix<double, 6, 6> &hessian,
				PointCloudSource &trans_cloud);

		













		bool
			updateIntervalMT(double &a_l, double &f_l, double &g_l,
				double &a_u, double &f_u, double &g_u,
				double a_t, double f_t, double g_t);

		















		double
			trialValueSelectionMT(double a_l, double f_l, double g_l,
				double a_u, double f_u, double g_u,
				double a_t, double f_t, double g_t);

		








		inline double
			auxiliaryFunction_PsiMT(double a, double f_a, double f_0, double g_0, double mu = 1.e-4)
		{
			return (f_a - f_0 - mu * g_0 * a);
		}

		






		inline double
			auxiliaryFunction_dPsiMT(double g_a, double g_0, double mu = 1.e-4)
		{
			return (g_a - mu * g_0);
		}

		
		TargetGrid target_cells_;

		

		
		float resolution_;

		
		double step_size_;

		
		double outlier_ratio_;

		
		double gauss_d1_, gauss_d2_, gauss_d3_;

		
		double trans_probability_;

		



		Eigen::Vector3d j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;

		Eigen::Matrix<float, 8, 4> j_ang;

		



		Eigen::Vector3d h_ang_a2_, h_ang_a3_,
			h_ang_b2_, h_ang_b3_,
			h_ang_c2_, h_ang_c3_,
			h_ang_d1_, h_ang_d2_, h_ang_d3_,
			h_ang_e1_, h_ang_e2_, h_ang_e3_,
			h_ang_f1_, h_ang_f2_, h_ang_f3_;

		Eigen::Matrix<float, 16, 4> h_ang;

		
  

		
  

    int num_threads_;

	Eigen::Matrix<double, 6, 6> hessian_;
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transformation_array_;
	double nearest_voxel_transformation_likelihood_;

	float regularization_scale_factor_;
	boost::optional<Eigen::Matrix4f> regularization_pose_;
	Eigen::Vector3f regularization_pose_translation_;

	public:
		NeighborSearchMethod search_method;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

}

#endif 
