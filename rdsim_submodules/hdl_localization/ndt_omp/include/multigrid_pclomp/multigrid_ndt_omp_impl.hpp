#include "multigrid_pclomp/multigrid_ndt_omp.h"






















































#ifndef PCL_REGISTRATION_NDT_OMP_MULTI_VOXEL_IMPL_H_
#define PCL_REGISTRATION_NDT_OMP_MULTI_VOXEL_IMPL_H_


template<typename PointSource, typename PointTarget>
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::MultiGridNormalDistributionsTransform ()
  : target_cells_ ()
  , resolution_ (1.0f)
  , step_size_ (0.1)
  , outlier_ratio_ (0.55)
  , gauss_d1_ ()
  , gauss_d2_ ()
  , gauss_d3_ ()
  , trans_probability_ ()
  , regularization_pose_ (boost::none)
  , j_ang_a_ (), j_ang_b_ (), j_ang_c_ (), j_ang_d_ (), j_ang_e_ (), j_ang_f_ (), j_ang_g_ (), j_ang_h_ ()
  , h_ang_a2_ (), h_ang_a3_ (), h_ang_b2_ (), h_ang_b3_ (), h_ang_c2_ (), h_ang_c3_ (), h_ang_d1_ (), h_ang_d2_ ()
  , h_ang_d3_ (), h_ang_e1_ (), h_ang_e2_ (), h_ang_e3_ (), h_ang_f1_ (), h_ang_f2_ (), h_ang_f3_ ()
{
  reg_name_ = "MultiGridNormalDistributionsTransform";

  double gauss_c1, gauss_c2;

  
  gauss_c1 = 10.0 * (1 - outlier_ratio_);
  gauss_c2 = outlier_ratio_ / pow (resolution_, 3);
  gauss_d3_ = -log (gauss_c2);
  gauss_d1_ = -log ( gauss_c1 + gauss_c2 ) - gauss_d3_;
  gauss_d2_ = -2 * log ((-log ( gauss_c1 * exp ( -0.5 ) + gauss_c2 ) - gauss_d3_) / gauss_d1_);

  transformation_epsilon_ = 0.1;
  max_iterations_ = 35;

  num_threads_ = omp_get_max_threads();
}



template<typename PointSource, typename PointTarget> void
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess)
{
  nr_iterations_ = 0;
  converged_ = false;

  double gauss_c1, gauss_c2;

  
  gauss_c1 = 10 * (1 - outlier_ratio_);
  gauss_c2 = outlier_ratio_ / pow (resolution_, 3);
  gauss_d3_ = -log (gauss_c2);
  gauss_d1_ = -log ( gauss_c1 + gauss_c2 ) - gauss_d3_;
  gauss_d2_ = -2 * log ((-log ( gauss_c1 * exp ( -0.5 ) + gauss_c2 ) - gauss_d3_) / gauss_d1_);

  if (guess != Eigen::Matrix4f::Identity ())
  {
    
    final_transformation_ = guess;
    
    transformPointCloud (output, output, guess);
  }

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
  eig_transformation.matrix () = final_transformation_;
  transformation_array_.clear();
  transformation_array_.push_back(final_transformation_);

  
  Eigen::Matrix<double, 6, 1> p, delta_p, score_gradient;
  Eigen::Vector3f init_translation = eig_transformation.translation ();
  Eigen::Vector3f init_rotation = eig_transformation.rotation ().eulerAngles (0, 1, 2);
  p << init_translation (0), init_translation (1), init_translation (2),
  init_rotation (0), init_rotation (1), init_rotation (2);

  Eigen::Matrix<double, 6, 6> hessian;

  double score = 0;
  double delta_p_norm;

  if (regularization_pose_)
  {
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> regularization_pose_transformation;
    regularization_pose_transformation.matrix() = regularization_pose_.get();
    regularization_pose_translation_ = regularization_pose_transformation.translation();
  }

  
  score = computeDerivatives (score_gradient, hessian, output, p);

  while (!converged_)
  {
    
    previous_transformation_ = transformation_;

    
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6> > sv (hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    delta_p = sv.solve (-score_gradient);

    
    delta_p_norm = delta_p.norm ();

    if (delta_p_norm == 0 || delta_p_norm != delta_p_norm)
    {
      if (input_->points.empty()) {
        trans_probability_ = 0.0f;
      }
      else {
        trans_probability_ = score / static_cast<double> (input_->points.size ());
      }

      converged_ = delta_p_norm == delta_p_norm;
      return;
    }

    delta_p.normalize ();
    delta_p_norm = computeStepLengthMT (p, delta_p, delta_p_norm, step_size_, transformation_epsilon_ / 2, score, score_gradient, hessian, output);
    delta_p *= delta_p_norm;


    transformation_ = (Eigen::Translation<float, 3> (static_cast<float> (delta_p (0)), static_cast<float> (delta_p (1)), static_cast<float> (delta_p (2))) *
                       Eigen::AngleAxis<float> (static_cast<float> (delta_p (3)), Eigen::Vector3f::UnitX ()) *
                       Eigen::AngleAxis<float> (static_cast<float> (delta_p (4)), Eigen::Vector3f::UnitY ()) *
                       Eigen::AngleAxis<float> (static_cast<float> (delta_p (5)), Eigen::Vector3f::UnitZ ())).matrix ();

    transformation_array_.push_back(final_transformation_);

    p = p + delta_p;

    
    if (update_visualizer_ != 0)
      update_visualizer_ (output, std::vector<int>(), *target_, std::vector<int>() );

    if (nr_iterations_ > max_iterations_ ||
        (nr_iterations_ && (std::fabs (delta_p_norm) < transformation_epsilon_)))
    {
      converged_ = true;
    }

    nr_iterations_++;

  }

  
  
  if (input_->points.empty()) {
    trans_probability_ = 0.0f;
  }
  else {
    trans_probability_ = score / static_cast<double> (input_->points.size ());
  }

  hessian_ = hessian;
}

#ifndef _OPENMP
int omp_get_max_threads() { return 1; }
int omp_get_thread_num() { return 0; }
#endif


template<typename PointSource, typename PointTarget> double
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient,
	Eigen::Matrix<double, 6, 6> &hessian,
	PointCloudSource &trans_cloud,
	Eigen::Matrix<double, 6, 1> &p,
	bool compute_hessian)
{
	score_gradient.setZero();
	hessian.setZero();
	double score = 0;
	int total_neighborhood_count = 0;
  double nearest_voxel_score = 0;
  size_t found_neigborhood_voxel_num = 0;

  std::vector<double> scores(input_->points.size());
  std::vector<double> nearest_voxel_scores(input_->points.size());
  std::vector<size_t> found_neigborhood_voxel_nums(input_->points.size());
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> score_gradients(input_->points.size());
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> hessians(input_->points.size());
  std::vector<int> neighborhood_counts(input_->points.size());
  for (std::size_t i = 0; i < input_->points.size(); i++) {
		scores[i] = 0;
    nearest_voxel_scores[i] = 0;
    found_neigborhood_voxel_nums[i] = 0;
		score_gradients[i].setZero();
		hessians[i].setZero();
		neighborhood_counts[i] = 0;
	}

	
	computeAngleDerivatives(p);

  std::vector<std::vector<TargetGridLeafConstPtr>> neighborhoods(num_threads_);
  std::vector<std::vector<float>> distancess(num_threads_);

  
#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
	for (std::size_t idx = 0; idx < input_->points.size(); idx++)
	{
		int thread_n = omp_get_thread_num();

		
		PointSource x_pt, x_trans_pt;
		
		Eigen::Vector3d x, x_trans;
		
		TargetGridLeafConstPtr cell;
		
		Eigen::Matrix3d c_inv;

		
		Eigen::Matrix<float, 4, 6> point_gradient_;
		Eigen::Matrix<float, 24, 6> point_hessian_;
		point_gradient_.setZero();
		point_gradient_.block<3, 3>(0, 0).setIdentity();
		point_hessian_.setZero();

		x_trans_pt = trans_cloud.points[idx];

		auto& neighborhood = neighborhoods[thread_n];
		auto& distances = distancess[thread_n];

    
    target_cells_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);

		double sum_score_pt = 0;
    double nearest_voxel_score_pt = 0;
		Eigen::Matrix<double, 6, 1> score_gradient_pt = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 6> hessian_pt = Eigen::Matrix<double, 6, 6>::Zero();
		int neighborhood_count = 0;

		for (typename std::vector<TargetGridLeafConstPtr>::iterator neighborhood_it = neighborhood.begin(); neighborhood_it != neighborhood.end(); neighborhood_it++)
		{
			cell = *neighborhood_it;
			x_pt = input_->points[idx];
			x = Eigen::Vector3d(x_pt.x, x_pt.y, x_pt.z);

			x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

			
			x_trans -= cell->getMean();
			
			c_inv = cell->getInverseCov();

			
			computePointDerivatives(x, point_gradient_, point_hessian_);
			
			double score_pt = updateDerivatives(score_gradient_pt, hessian_pt, point_gradient_, point_hessian_, x_trans, c_inv, compute_hessian);
			neighborhood_count++;
      sum_score_pt += score_pt;
      if (score_pt > nearest_voxel_score_pt) {
        nearest_voxel_score_pt = score_pt;
      }
		}

    if(!neighborhood.empty()) {
      ++found_neigborhood_voxel_nums[idx];
    }

		scores[idx] = sum_score_pt;
    nearest_voxel_scores[idx] = nearest_voxel_score_pt;
		score_gradients[idx].noalias() = score_gradient_pt;
		hessians[idx].noalias() = hessian_pt;
		neighborhood_counts[idx] += neighborhood_count;
	}

  
  for (std::size_t i = 0; i < input_->points.size(); i++) {
		score += scores[i];
    nearest_voxel_score += nearest_voxel_scores[i];
    found_neigborhood_voxel_num += found_neigborhood_voxel_nums[i];
		score_gradient += score_gradients[i];
		hessian += hessians[i];
		total_neighborhood_count += neighborhood_counts[i];
	}

	if (regularization_pose_) {
		float regularization_score = 0.0f;
		Eigen::Matrix<double, 6, 1> regularization_gradient = Eigen::Matrix<double, 6, 1>::Zero();
		Eigen::Matrix<double, 6, 6> regularization_hessian = Eigen::Matrix<double, 6, 6>::Zero();

		const float dx = regularization_pose_translation_(0) - static_cast<float>(p(0, 0));
		const float dy = regularization_pose_translation_(1) - static_cast<float>(p(1, 0));
		const auto sin_yaw = static_cast<float>(sin(p(5, 0)));
		const auto cos_yaw = static_cast<float>(cos(p(5, 0)));
		const float longitudinal_distance = dy * sin_yaw + dx * cos_yaw;
		const auto neighborhood_count_weight = static_cast<float>(total_neighborhood_count);

		regularization_score = - regularization_scale_factor_ * neighborhood_count_weight * longitudinal_distance * longitudinal_distance;

		regularization_gradient(0, 0) = regularization_scale_factor_ * neighborhood_count_weight * 2.0f * cos_yaw * longitudinal_distance;
		regularization_gradient(1, 0) = regularization_scale_factor_ * neighborhood_count_weight * 2.0f * sin_yaw * longitudinal_distance;

		regularization_hessian(0, 0) = - regularization_scale_factor_ * neighborhood_count_weight * 2.0f * cos_yaw * cos_yaw;
		regularization_hessian(0, 1) = - regularization_scale_factor_ * neighborhood_count_weight * 2.0f * cos_yaw * sin_yaw;
		regularization_hessian(1, 1) = - regularization_scale_factor_ * neighborhood_count_weight * 2.0f * sin_yaw * sin_yaw;
		regularization_hessian(1, 0) = regularization_hessian(0, 1);

		score += regularization_score;
		score_gradient += regularization_gradient;
		hessian += regularization_hessian;
	}

  if (found_neigborhood_voxel_num != 0) {
    nearest_voxel_transformation_likelihood_ = nearest_voxel_score / static_cast<double>(found_neigborhood_voxel_num);
  }
  else {
    nearest_voxel_transformation_likelihood_ = 0.0;
  }

	return (score);
}


template<typename PointSource, typename PointTarget> void
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::computeAngleDerivatives(Eigen::Matrix<double, 6, 1> &p, bool compute_hessian)
{
	
	double cx, cy, cz, sx, sy, sz;
	if (fabs(p(3)) < 10e-5)
	{
		
		cx = 1.0;
		sx = 0.0;
	}
	else
	{
		cx = cos(p(3));
		sx = sin(p(3));
	}
	if (fabs(p(4)) < 10e-5)
	{
		
		cy = 1.0;
		sy = 0.0;
	}
	else
	{
		cy = cos(p(4));
		sy = sin(p(4));
	}

	if (fabs(p(5)) < 10e-5)
	{
		
		cz = 1.0;
		sz = 0.0;
	}
	else
	{
		cz = cos(p(5));
		sz = sin(p(5));
	}

	
	j_ang_a_ << (-sx * sz + cx * sy * cz), (-sx * cz - cx * sy * sz), (-cx * cy);
	j_ang_b_ << (cx * sz + sx * sy * cz), (cx * cz - sx * sy * sz), (-sx * cy);
	j_ang_c_ << (-sy * cz), sy * sz, cy;
	j_ang_d_ << sx * cy * cz, (-sx * cy * sz), sx * sy;
	j_ang_e_ << (-cx * cy * cz), cx * cy * sz, (-cx * sy);
	j_ang_f_ << (-cy * sz), (-cy * cz), 0;
	j_ang_g_ << (cx * cz - sx * sy * sz), (-cx * sz - sx * sy * cz), 0;
	j_ang_h_ << (sx * cz + cx * sy * sz), (cx * sy * cz - sx * sz), 0;

	j_ang.setZero();
	j_ang.row(0).noalias() = Eigen::Vector4f((-sx * sz + cx * sy * cz), (-sx * cz - cx * sy * sz), (-cx * cy), 0.0f);
	j_ang.row(1).noalias() = Eigen::Vector4f((cx * sz + sx * sy * cz), (cx * cz - sx * sy * sz), (-sx * cy), 0.0f);
	j_ang.row(2).noalias() = Eigen::Vector4f((-sy * cz), sy * sz, cy, 0.0f);
	j_ang.row(3).noalias() = Eigen::Vector4f(sx * cy * cz, (-sx * cy * sz), sx * sy, 0.0f);
	j_ang.row(4).noalias() = Eigen::Vector4f((-cx * cy * cz), cx * cy * sz, (-cx * sy), 0.0f);
	j_ang.row(5).noalias() = Eigen::Vector4f((-cy * sz), (-cy * cz), 0, 0.0f);
	j_ang.row(6).noalias() = Eigen::Vector4f((cx * cz - sx * sy * sz), (-cx * sz - sx * sy * cz), 0, 0.0f);
	j_ang.row(7).noalias() = Eigen::Vector4f((sx * cz + cx * sy * sz), (cx * sy * cz - sx * sz), 0, 0.0f);

	if (compute_hessian)
	{
		
		h_ang_a2_ << (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), sx * cy;
		h_ang_a3_ << (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), (-cx * cy);

		h_ang_b2_ << (cx * cy * cz), (-cx * cy * sz), (cx * sy);
		h_ang_b3_ << (sx * cy * cz), (-sx * cy * sz), (sx * sy);

		h_ang_c2_ << (-sx * cz - cx * sy * sz), (sx * sz - cx * sy * cz), 0;
		h_ang_c3_ << (cx * cz - sx * sy * sz), (-sx * sy * cz - cx * sz), 0;

		h_ang_d1_ << (-cy * cz), (cy * sz), (sy);
		h_ang_d2_ << (-sx * sy * cz), (sx * sy * sz), (sx * cy);
		h_ang_d3_ << (cx * sy * cz), (-cx * sy * sz), (-cx * cy);

		h_ang_e1_ << (sy * sz), (sy * cz), 0;
		h_ang_e2_ << (-sx * cy * sz), (-sx * cy * cz), 0;
		h_ang_e3_ << (cx * cy * sz), (cx * cy * cz), 0;

		h_ang_f1_ << (-cy * cz), (cy * sz), 0;
		h_ang_f2_ << (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), 0;
		h_ang_f3_ << (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), 0;

		h_ang.setZero();
		h_ang.row(0).noalias() = Eigen::Vector4f((-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), sx * cy, 0.0f);		
		h_ang.row(1).noalias() = Eigen::Vector4f((-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), (-cx * cy), 0.0f);	

		h_ang.row(2).noalias() = Eigen::Vector4f((cx * cy * cz), (-cx * cy * sz), (cx * sy), 0.0f);							
		h_ang.row(3).noalias() = Eigen::Vector4f((sx * cy * cz), (-sx * cy * sz), (sx * sy), 0.0f);							

		h_ang.row(4).noalias() = Eigen::Vector4f((-sx * cz - cx * sy * sz), (sx * sz - cx * sy * cz), 0, 0.0f);				
		h_ang.row(5).noalias() = Eigen::Vector4f((cx * cz - sx * sy * sz), (-sx * sy * cz - cx * sz), 0, 0.0f);				

		h_ang.row(6).noalias() = Eigen::Vector4f((-cy * cz), (cy * sz), (sy), 0.0f);										
		h_ang.row(7).noalias() = Eigen::Vector4f((-sx * sy * cz), (sx * sy * sz), (sx * cy), 0.0f);							
		h_ang.row(8).noalias() = Eigen::Vector4f((cx * sy * cz), (-cx * sy * sz), (-cx * cy), 0.0f);						

		h_ang.row(9).noalias() = Eigen::Vector4f((sy * sz), (sy * cz), 0, 0.0f);											
		h_ang.row(10).noalias() = Eigen::Vector4f ((-sx * cy * sz), (-sx * cy * cz), 0, 0.0f);								
		h_ang.row(11).noalias() = Eigen::Vector4f ((cx * cy * sz), (cx * cy * cz), 0, 0.0f);								

		h_ang.row(12).noalias() = Eigen::Vector4f ((-cy * cz), (cy * sz), 0, 0.0f);											
		h_ang.row(13).noalias() = Eigen::Vector4f ((-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), 0, 0.0f);			
		h_ang.row(14).noalias() = Eigen::Vector4f ((-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), 0, 0.0f);			
	}
}


template<typename PointSource, typename PointTarget> void
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::computePointDerivatives(Eigen::Vector3d &x, Eigen::Matrix<float, 4, 6>& point_gradient_, Eigen::Matrix<float, 24, 6>& point_hessian_, bool compute_hessian) const
{
	Eigen::Vector4f x4(x[0], x[1], x[2], 0.0f);

	
	
	Eigen::Matrix<float, 8, 1> x_j_ang = j_ang * x4;

	point_gradient_(1, 3) = x_j_ang[0];
	point_gradient_(2, 3) = x_j_ang[1];
	point_gradient_(0, 4) = x_j_ang[2];
	point_gradient_(1, 4) = x_j_ang[3];
	point_gradient_(2, 4) = x_j_ang[4];
	point_gradient_(0, 5) = x_j_ang[5];
	point_gradient_(1, 5) = x_j_ang[6];
	point_gradient_(2, 5) = x_j_ang[7];

	if (compute_hessian)
	{
		Eigen::Matrix<float, 16, 1> x_h_ang = h_ang * x4;

		
		Eigen::Vector4f a (0, x_h_ang[0], x_h_ang[1], 0.0f);
		Eigen::Vector4f b (0, x_h_ang[2], x_h_ang[3], 0.0f);
		Eigen::Vector4f c (0, x_h_ang[4], x_h_ang[5], 0.0f);
		Eigen::Vector4f d (x_h_ang[6], x_h_ang[7], x_h_ang[8], 0.0f);
		Eigen::Vector4f e (x_h_ang[9], x_h_ang[10], x_h_ang[11], 0.0f);
		Eigen::Vector4f f (x_h_ang[12], x_h_ang[13], x_h_ang[14], 0.0f);

		
		
		point_hessian_.block<4, 1>((9/3)*4, 3) = a;
		point_hessian_.block<4, 1>((12/3)*4, 3) = b;
		point_hessian_.block<4, 1>((15/3)*4, 3) = c;
		point_hessian_.block<4, 1>((9/3)*4, 4) = b;
		point_hessian_.block<4, 1>((12/3)*4, 4) = d;
		point_hessian_.block<4, 1>((15/3)*4, 4) = e;
		point_hessian_.block<4, 1>((9/3)*4, 5) = c;
		point_hessian_.block<4, 1>((12/3)*4, 5) = e;
		point_hessian_.block<4, 1>((15/3)*4, 5) = f;
	}
}


template<typename PointSource, typename PointTarget> void
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::computePointDerivatives(Eigen::Vector3d &x, Eigen::Matrix<double, 3, 6>& point_gradient_, Eigen::Matrix<double, 18, 6>& point_hessian_, bool compute_hessian) const
{
	
	
	point_gradient_(1, 3) = x.dot(j_ang_a_);
	point_gradient_(2, 3) = x.dot(j_ang_b_);
	point_gradient_(0, 4) = x.dot(j_ang_c_);
	point_gradient_(1, 4) = x.dot(j_ang_d_);
	point_gradient_(2, 4) = x.dot(j_ang_e_);
	point_gradient_(0, 5) = x.dot(j_ang_f_);
	point_gradient_(1, 5) = x.dot(j_ang_g_);
	point_gradient_(2, 5) = x.dot(j_ang_h_);

	if (compute_hessian)
	{
		
		Eigen::Vector3d a, b, c, d, e, f;

		a << 0, x.dot(h_ang_a2_), x.dot(h_ang_a3_);
		b << 0, x.dot(h_ang_b2_), x.dot(h_ang_b3_);
		c << 0, x.dot(h_ang_c2_), x.dot(h_ang_c3_);
		d << x.dot(h_ang_d1_), x.dot(h_ang_d2_), x.dot(h_ang_d3_);
		e << x.dot(h_ang_e1_), x.dot(h_ang_e2_), x.dot(h_ang_e3_);
		f << x.dot(h_ang_f1_), x.dot(h_ang_f2_), x.dot(h_ang_f3_);

		
		
		point_hessian_.block<3, 1>(9, 3) = a;
		point_hessian_.block<3, 1>(12, 3) = b;
		point_hessian_.block<3, 1>(15, 3) = c;
		point_hessian_.block<3, 1>(9, 4) = b;
		point_hessian_.block<3, 1>(12, 4) = d;
		point_hessian_.block<3, 1>(15, 4) = e;
		point_hessian_.block<3, 1>(9, 5) = c;
		point_hessian_.block<3, 1>(12, 5) = e;
		point_hessian_.block<3, 1>(15, 5) = f;
	}
}


template<typename PointSource, typename PointTarget> double
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::updateDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient,
	Eigen::Matrix<double, 6, 6> &hessian,
	const Eigen::Matrix<float, 4, 6> &point_gradient4,
	const Eigen::Matrix<float, 24, 6> &point_hessian_,
	const Eigen::Vector3d &x_trans, const Eigen::Matrix3d &c_inv,
	bool compute_hessian) const
{
	Eigen::Matrix<float, 1, 4> x_trans4( x_trans[0], x_trans[1], x_trans[2], 0.0f );
	Eigen::Matrix4f c_inv4 = Eigen::Matrix4f::Zero();
	c_inv4.topLeftCorner(3, 3) = c_inv.cast<float>();

	float gauss_d2 = gauss_d2_;

	
	float e_x_cov_x = exp(-gauss_d2 * x_trans4.dot(x_trans4 * c_inv4) * 0.5f);
	
	float score_inc = -gauss_d1_ * e_x_cov_x;

	e_x_cov_x = gauss_d2 * e_x_cov_x;

	
	if (e_x_cov_x > 1 || e_x_cov_x < 0 || e_x_cov_x != e_x_cov_x)
		return (0);

	
	e_x_cov_x *= gauss_d1_;

	Eigen::Matrix<float, 4, 6> c_inv4_x_point_gradient4 = c_inv4 * point_gradient4;
	Eigen::Matrix<float, 6, 1> x_trans4_dot_c_inv4_x_point_gradient4 = x_trans4 * c_inv4_x_point_gradient4;

	score_gradient.noalias() += (e_x_cov_x * x_trans4_dot_c_inv4_x_point_gradient4).cast<double>();

	if (compute_hessian) {
		Eigen::Matrix<float, 1, 4> x_trans4_x_c_inv4 = x_trans4 * c_inv4;
		Eigen::Matrix<float, 6, 6> point_gradient4_colj_dot_c_inv4_x_point_gradient4_col_i = point_gradient4.transpose() * c_inv4_x_point_gradient4;
		Eigen::Matrix<float, 6, 1> x_trans4_dot_c_inv4_x_ext_point_hessian_4ij;

		for (int i = 0; i < 6; i++) {
			
			
			x_trans4_dot_c_inv4_x_ext_point_hessian_4ij.noalias() = x_trans4_x_c_inv4 * point_hessian_.block<4, 6>(i * 4, 0);

			for (int j = 0; j < hessian.cols(); j++) {
				
				hessian(i, j) += e_x_cov_x * (-gauss_d2 * x_trans4_dot_c_inv4_x_point_gradient4(i) * x_trans4_dot_c_inv4_x_point_gradient4(j) +
					x_trans4_dot_c_inv4_x_ext_point_hessian_4ij(j) +
					point_gradient4_colj_dot_c_inv4_x_point_gradient4_col_i(j, i));
			}
		}
	}

	return (score_inc);
}


template<typename PointSource, typename PointTarget> void
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::computeHessian (Eigen::Matrix<double, 6, 6> &hessian,
                                                                             PointCloudSource &trans_cloud, Eigen::Matrix<double, 6, 1> &)
{
  
  PointSource x_pt, x_trans_pt;
  
  Eigen::Vector3d x, x_trans;
  
  TargetGridLeafConstPtr cell;
  
  Eigen::Matrix3d c_inv;

  
  Eigen::Matrix<double, 3, 6> point_gradient_;
  Eigen::Matrix<double, 18, 6> point_hessian_;
  point_gradient_.setZero();
  point_gradient_.block<3, 3>(0, 0).setIdentity();
  point_hessian_.setZero();

  hessian.setZero ();

  

  
  for (size_t idx = 0; idx < input_->points.size (); idx++)
  {
    x_trans_pt = trans_cloud.points[idx];

    
    std::vector<TargetGridLeafConstPtr> neighborhood;
    std::vector<float> distances;

    
    target_cells_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);

    for (typename std::vector<TargetGridLeafConstPtr>::iterator neighborhood_it = neighborhood.begin (); neighborhood_it != neighborhood.end (); neighborhood_it++)
    {
      cell = *neighborhood_it;

      {
        x_pt = input_->points[idx];
        x = Eigen::Vector3d (x_pt.x, x_pt.y, x_pt.z);

        x_trans = Eigen::Vector3d (x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

        
        x_trans -= cell->getMean ();
        
        c_inv = cell->getInverseCov ();

        
        computePointDerivatives (x, point_gradient_, point_hessian_);
        
        updateHessian (hessian, point_gradient_, point_hessian_, x_trans, c_inv);
      }
    }
  }
}


template<typename PointSource, typename PointTarget> void
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::updateHessian (Eigen::Matrix<double, 6, 6> &hessian,
	const Eigen::Matrix<double, 3, 6> &point_gradient_,
	const Eigen::Matrix<double, 18, 6> &point_hessian_,
	const Eigen::Vector3d &x_trans,
	const Eigen::Matrix3d &c_inv) const
{
  Eigen::Vector3d cov_dxd_pi;
  
  double e_x_cov_x = gauss_d2_ * exp (-gauss_d2_ * x_trans.dot (c_inv * x_trans) / 2);

  
  if (e_x_cov_x > 1 || e_x_cov_x < 0 || e_x_cov_x != e_x_cov_x)
    return;

  
  e_x_cov_x *= gauss_d1_;

  for (int i = 0; i < 6; i++)
  {
    
    cov_dxd_pi = c_inv * point_gradient_.col (i);

    for (int j = 0; j < hessian.cols (); j++)
    {
      
      hessian (i, j) += e_x_cov_x * (-gauss_d2_ * x_trans.dot (cov_dxd_pi) * x_trans.dot (c_inv * point_gradient_.col (j)) +
                                  x_trans.dot (c_inv * point_hessian_.block<3, 1>(3 * i, j)) +
                                  point_gradient_.col (j).dot (cov_dxd_pi) );
    }
  }

}


template<typename PointSource, typename PointTarget> bool
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::updateIntervalMT (double &a_l, double &f_l, double &g_l,
                                                                               double &a_u, double &f_u, double &g_u,
                                                                               double a_t, double f_t, double g_t)
{
  
  if (f_t > f_l)
  {
    a_u = a_t;
    f_u = f_t;
    g_u = g_t;
    return (false);
  }
  
  else
  if (g_t * (a_l - a_t) > 0)
  {
    a_l = a_t;
    f_l = f_t;
    g_l = g_t;
    return (false);
  }
  
  else
  if (g_t * (a_l - a_t) < 0)
  {
    a_u = a_l;
    f_u = f_l;
    g_u = g_l;

    a_l = a_t;
    f_l = f_t;
    g_l = g_t;
    return (false);
  }
  
  else
    return (true);
}


template<typename PointSource, typename PointTarget> double
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::trialValueSelectionMT (double a_l, double f_l, double g_l,
                                                                                    double a_u, double f_u, double g_u,
                                                                                    double a_t, double f_t, double g_t)
{
  
  if (f_t > f_l)
  {
    
    
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt (z * z - g_t * g_l);
    
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    
    
    double a_q = a_l - 0.5 * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

    if (std::fabs (a_c - a_l) < std::fabs (a_q - a_l))
      return (a_c);
    else
      return (0.5 * (a_q + a_c));
  }
  
  else
  if (g_t * g_l < 0)
  {
    
    
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt (z * z - g_t * g_l);
    
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    
    
    double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

    if (std::fabs (a_c - a_t) >= std::fabs (a_s - a_t))
      return (a_c);
    else
      return (a_s);
  }
  
  else
  if (std::fabs (g_t) <= std::fabs (g_l))
  {
    
    
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt (z * z - g_t * g_l);
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    
    
    double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

    double a_t_next;

    if (std::fabs (a_c - a_t) < std::fabs (a_s - a_t))
      a_t_next = a_c;
    else
      a_t_next = a_s;

    if (a_t > a_l)
      return (std::min (a_t + 0.66 * (a_u - a_t), a_t_next));
    else
      return (std::max (a_t + 0.66 * (a_u - a_t), a_t_next));
  }
  
  else
  {
    
    
    double z = 3 * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
    double w = std::sqrt (z * z - g_t * g_u);
    
    return (a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2 * w));
  }
}


template<typename PointSource, typename PointTarget> double
pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::computeStepLengthMT (const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &step_dir, double step_init, double step_max,
                                                                                  double step_min, double &score, Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
                                                                                  PointCloudSource &trans_cloud)
{
  
  double phi_0 = -score;
  
  double d_phi_0 = -(score_gradient.dot (step_dir));

  Eigen::Matrix<double, 6, 1>  x_t;

  if (d_phi_0 >= 0)
  {
    
    if (d_phi_0 == 0)
      return 0;
    else
    {
      
      d_phi_0 *= -1;
      step_dir *= -1;

    }
  }

  

  int max_step_iterations = 10;
  int step_iterations = 0;

  
  double mu = 1.e-4;
  
  double nu = 0.9;

  
  double a_l = 0, a_u = 0;

  
  double f_l = auxiliaryFunction_PsiMT (a_l, phi_0, phi_0, d_phi_0, mu);
  double g_l = auxiliaryFunction_dPsiMT (d_phi_0, d_phi_0, mu);

  double f_u = auxiliaryFunction_PsiMT (a_u, phi_0, phi_0, d_phi_0, mu);
  double g_u = auxiliaryFunction_dPsiMT (d_phi_0, d_phi_0, mu);

  
  bool interval_converged = (step_max - step_min) < 0, open_interval = true;

  double a_t = step_init;
  a_t = std::min (a_t, step_max);
  a_t = std::max (a_t, step_min);

  x_t = x + step_dir * a_t;

  final_transformation_ = (Eigen::Translation<float, 3>(static_cast<float> (x_t (0)), static_cast<float> (x_t (1)), static_cast<float> (x_t (2))) *
                           Eigen::AngleAxis<float> (static_cast<float> (x_t (3)), Eigen::Vector3f::UnitX ()) *
                           Eigen::AngleAxis<float> (static_cast<float> (x_t (4)), Eigen::Vector3f::UnitY ()) *
                           Eigen::AngleAxis<float> (static_cast<float> (x_t (5)), Eigen::Vector3f::UnitZ ())).matrix ();

  
  transformPointCloud (*input_, trans_cloud, final_transformation_);

  
  
  score = computeDerivatives (score_gradient, hessian, trans_cloud, x_t, true);

  
  
  
  
  
  
  
  

  
  
  
  

  
  
  
  

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

  
  

  

  
  
  
  

  
  
  

  
  

  
  
  
  

  
  
  
  

  
  
  
  

  
  
  

  
  
  
  

  
  
  
  
  
  
  
  
  
  
  
  
  
  

  
  

  
  
  
  if (step_iterations)
    computeHessian (hessian, trans_cloud, x_t);

  return (a_t);
}

template<typename PointSource, typename PointTarget>
double pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::calculateScore(const PointCloudSource & trans_cloud) const
{
	double score = 0;

	for (std::size_t idx = 0; idx < trans_cloud.points.size(); idx++)
	{
		PointSource x_trans_pt = trans_cloud.points[idx];

		
		std::vector<TargetGridLeafConstPtr> neighborhood;
		std::vector<float> distances;

    
    target_cells_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);

		for (typename std::vector<TargetGridLeafConstPtr>::iterator neighborhood_it = neighborhood.begin(); neighborhood_it != neighborhood.end(); neighborhood_it++)
		{
			TargetGridLeafConstPtr cell = *neighborhood_it;

			Eigen::Vector3d x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

			
			x_trans -= cell->getMean();
			
			Eigen::Matrix3d c_inv = cell->getInverseCov();

			
			double e_x_cov_x = exp(-gauss_d2_ * x_trans.dot(c_inv * x_trans) / 2);
			
			double score_inc = -gauss_d1_ * e_x_cov_x - gauss_d3_;

			score += score_inc / neighborhood.size();
		}
	}

  double output_score = 0;
  if (!trans_cloud.points.empty()) {
    output_score = (score) / static_cast<double> (trans_cloud.size());
  }
	return output_score;
}

template<typename PointSource, typename PointTarget>
double pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::calculateTransformationProbability(const PointCloudSource & trans_cloud) const
{
	double score = 0;

	for (std::size_t idx = 0; idx < trans_cloud.points.size(); idx++)
	{
		PointSource x_trans_pt = trans_cloud.points[idx];

		
		std::vector<TargetGridLeafConstPtr> neighborhood;
		std::vector<float> distances;

    
    target_cells_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);

		for (typename std::vector<TargetGridLeafConstPtr>::iterator neighborhood_it = neighborhood.begin(); neighborhood_it != neighborhood.end(); neighborhood_it++)
		{
			TargetGridLeafConstPtr cell = *neighborhood_it;

			Eigen::Vector3d x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

			
			x_trans -= cell->getMean();
			
			Eigen::Matrix3d c_inv = cell->getInverseCov();

			
			double e_x_cov_x = exp(-gauss_d2_ * x_trans.dot(c_inv * x_trans) / 2);
			
			double score_inc = -gauss_d1_ * e_x_cov_x;

      score += score_inc;
		}
	}

  double output_score = 0;
  if (!trans_cloud.points.empty()) {
    output_score = (score) / static_cast<double> (trans_cloud.points.size());
  }
	return output_score;
}

template<typename PointSource, typename PointTarget>
double pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>::calculateNearestVoxelTransformationLikelihood(const PointCloudSource & trans_cloud) const
{
  double nearest_voxel_score = 0;
  size_t found_neigborhood_voxel_num = 0;

	for (std::size_t idx = 0; idx < trans_cloud.points.size(); idx++)
	{
    double nearest_voxel_score_pt = 0;
		PointSource x_trans_pt = trans_cloud.points[idx];

		
		std::vector<TargetGridLeafConstPtr> neighborhood;
		std::vector<float> distances;

    
    target_cells_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);

		for (typename std::vector<TargetGridLeafConstPtr>::iterator neighborhood_it = neighborhood.begin(); neighborhood_it != neighborhood.end(); neighborhood_it++)
		{
			TargetGridLeafConstPtr cell = *neighborhood_it;

			Eigen::Vector3d x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

			
			x_trans -= cell->getMean();
			
			Eigen::Matrix3d c_inv = cell->getInverseCov();

			
			double e_x_cov_x = exp(-gauss_d2_ * x_trans.dot(c_inv * x_trans) / 2);
			
			double score_inc = -gauss_d1_ * e_x_cov_x;

      if (score_inc > nearest_voxel_score_pt) {
        nearest_voxel_score_pt = score_inc;
      }
		}

    if (!neighborhood.empty()) {
      ++found_neigborhood_voxel_num;
      nearest_voxel_score += nearest_voxel_score_pt;
    }

	}

  double output_score = 0;
  if (found_neigborhood_voxel_num != 0) {
    output_score =  nearest_voxel_score / static_cast<double> (found_neigborhood_voxel_num);
  }
  return output_score;
}

#endif 
