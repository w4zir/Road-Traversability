/*
 * RTI.h
 *
 *  Created on: Feb 16, 2016
 *      Author: mudassir
 */

#ifndef RTI_H_
#define RTI_H_


#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
//#include <pcl/search/search.h>
//#include <pcl/search/octree.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
//#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>
#include <math.h>
#include <time.h>
#include "dem.h"
#include <iostream>
#include <fstream>

namespace pcl
{
/** \brief
 * Implement Digital Elevation Map from road point cloud data.
 */
template <typename PointT>
class PCL_EXPORTS RTI : public DEM<PointT>
{
public:
	typedef pcl::octree::OctreePointCloudSearch<PointT> OcTree;
	typedef typename OcTree::Ptr OcTreePtr;
	typedef pcl::search::Search <PointT> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	typedef pcl::PointCloud <PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;

	using DEM<PointT>::debug_mode_;

	using DEM<PointT>::road_min_x_;
	using DEM<PointT>::road_max_x_;
	using DEM<PointT>::road_min_y_;
	using DEM<PointT>::road_max_y_;
	using DEM<PointT>::search_;
	using DEM<PointT>::dem_clusters_;
	using DEM<PointT>::cloud_dem_;
	using DEM<PointT>::cloud_project_;

	using DEM<PointT>::setRoadPlaneCoefficients;
	using DEM<PointT>::computeDEM;
	using DEM<PointT>::getDEMVisibilityCloud;
	using PCLBase <PointT>::initCompute;
	using PCLBase <PointT>::deinitCompute;
	//   using DEM<PointT>::cloud_project_;
	//   using DEM<PointT>::cloud_project_;
	//   using DEM<PointT>::input_;

public:

	/** \brief Constructor that sets default values for member variables. */
	RTI ();

	/** \brief This destructor destroys the cloud, normals and search method used for
	 * finding KNN. In other words it frees memory.
	 */
	virtual
	~RTI ();

	/** \brief prm and vehicle parameters. */
	void
	loadParameters();

	/** \brief Get current vehicle id. */
	virtual std::string
	getVehicleId () const;

	/** \brief Set current vehicle id. */
	virtual void
	setVehicleId (std::string vehicle_id);

	/** \brief Set goal biad probability. */
	virtual void
	setRandomConfigsFlag (bool is_random_configs);

	/** \brief Set goal biad probability. */
	virtual bool
	getImportConfigsFlag () const;

	/** \brief Set goal biad probability. */
	virtual void
	setImportConfigsFlag (bool is_import_configs);

	/** \brief Set goal biad probability. */
	virtual bool
	getRandomConfigsFlag () const;

	/** \brief Set goal biad probability. */
	virtual void
	setGoalBiasProbability (float goal_bias);

	/** \brief Set goal biad probability. */
	virtual float
	getGoalBiasProbability () const;

	/** \brief Set known obstacles info. */
	virtual void
	setObstaclesInfo (Eigen::MatrixXf &obstacles_info);

	/** \brief  Get known obstacles info. */
	virtual void
	getObstaclesInfo (Eigen::MatrixXf &obstacles_info);

	/** \brief Set goal biad probability. */
	virtual int
	getConfigCounter () const;

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	std::vector<int>
	getConfigurationsValidityStatus ();

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	std::vector<float>
	getConfigurationsClearance ();

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	Eigen::MatrixXf
	getConfigurations ();

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	virtual void
	getConfigurationsOnly (Eigen::MatrixXf &vehicle_configs);

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	virtual void
	setConfigurations (Eigen::MatrixXf vehicle_configs);

	/** \brief Get ratios of angles of two configs making an edge of prm graph. */
	virtual void
	getEdgeAngelRatios (std::vector<float> &angle_ratio);

	/** \brief Returns dimension of RTI in y direction. */
	void
	computeRTI ();

	/** \brief Get the DEM clusters. */
	virtual void
	getRTIAdjacencyList (std::vector< std::pair<int,int> >& prm_graph);

	/** \brief Cloud for visibility of RTI. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	getRTIVisibilityCloud();

protected:

	/** \brief Allows to set search method that will be used for finding KNN.
	 * \param[in] search search method to use
	 */
	void
	setDEMSearchMethod (const OcTreePtr& tree);

	/** \brief Generate random vehicle configuration. */
	virtual Eigen::Vector3f
	generateRandomConfiguration ();

	/** \brief Find nearest config to the random config. */
	virtual int
	findQrandNearestNeighbor (Eigen::Vector3f qrand);

	/** \brief Find distance between two configurations. */
	virtual float
	config_dist (Eigen::Vector3f qrand, Eigen::Vector4f qnear);

	/** \brief Find new configuration closer to q_rand from q_near. */
	virtual Eigen::Vector4f
	findOptimalConfig (Eigen::Vector3f qrand, Eigen::Vector4f qnear);

	/** \brief Check whether q_new is safe or not. */
	virtual int
	checkConfigSafety (Eigen::Vector4f config);

	/** \brief Returns dimension of RTI in y direction. */
	virtual void
	addStartConfigurations();

	/** \brief Generate PRM cloud from matrix. Used for search for neighbors configurations. */
	virtual void
	getConfigCloud ();

	/** \brief find configs neighbor index. */
	virtual void
	findConfigNeighbours ();

	/** \brief Basic version of computing sub-configs between current and neighbor config. */
	virtual Eigen::MatrixXf
	getNeighborSubConfigsBasic(Eigen::Vector4f c_config, Eigen::Vector4f n_config);

	/** \brief Computing sub-configs between current and neighbor config. */
	virtual Eigen::MatrixXf
	getNeighborSubConfigs(Eigen::Vector4f c_config, Eigen::Vector4f n_config, float neighbor_ratio);

	/** \brief Compute circular motion info of a vehicle config. */
	virtual Eigen::MatrixXf
	getConfigCircluarCenters (Eigen::Vector4f config);

	/** \brief Returns dimension of RTI in y direction. */
	virtual void
	generatePRMGraph();

	/** \brief Returns dimension of RTI in y direction. */
	virtual void
	validateConfigurations ();

	/** \brief Returns dimension of RTI in y direction. */
	virtual void
	clearanceUsingInvalidConfigs ();

	/** \brief Basic version of checking whether two configs are reachable or not. */
	virtual float
	findConfigsConnectivityBasic (Eigen::Vector4f c_config, Eigen::Vector4f n_config);

	/** \brief Check whether two configs are reachable or not. */
	virtual bool
	findConfigsConnectivity (Eigen::Vector4f c_config, Eigen::Vector4f n_config, Eigen::MatrixXf config_circles);

	/** \brief Check whether two configs are reachable or not. */
	virtual float
	findConfigsConnectivity (Eigen::Vector4f c_config, Eigen::Vector4f n_config);

	/** \brief Generate all vehicle configurations. */
	virtual void
	generateConfigurations ();

	/** \brief Returns dimension of PRM in y direction. */
	virtual void
	generateQuasiRandomConfigurations();

	/** \brief Returns dimension of PRM in y direction. */
	virtual void
	generateRandomConfigurations();

	/** \brief Generate all vehicle configurations. */
	virtual bool
	collisionChecker(Eigen::MatrixXf vehicle_state);

	/** \brief Collision checker for vehicle state. */
	virtual int
	collisionAndSafetyChecker(Eigen::MatrixXf vehicle_state);

	/** \brief Generate all vehicle configurations. */
	virtual float
	computeConfigClearance(Eigen::Vector3f c_config, Eigen::MatrixXf vehicle_state);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual bool
	checkDemUnderVehicle (int d_idx, Eigen::MatrixXf vehicle_body_state);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual bool
	checkPointUnderVehicle (Eigen::Vector3f p, Eigen::MatrixXf vehicle_body_state);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual bool
	checkCircleCollisdeWithRectangle (Eigen::Vector3f circle, Eigen::MatrixXf vehicle_body_state);

	/** \brief Generate default DEM cells under vehicle body at (0,0,0). */
	virtual int
	collisionCheckerWithKnownObstacles (Eigen::MatrixXf vehicle_configs);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual bool
	checkDemInsideTiresBoundingBoxes (int d_idx, Eigen::MatrixXf clearance_bounding_boxes);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual float
	computePositiveObstacleDistance (int d_idx, Eigen::MatrixXf vehicle_body_state);

	/** \brief Check whether dem clusters is under the body of vehicle or not. */
	virtual float
	computeNegativeObstacleDistance (int d_idx, Eigen::MatrixXf vehicle_body_state);

	/** \brief Check dem clusters under tyre for collision with vehicle. */
	virtual bool
	tyreClusterCollisionProcessing(int d_idx);

	/** \brief Check dem clusters under body for collision with vehicle. */
	virtual bool
	bodyClusterCollisionProcessing(int d_idx);


	/** \brief Returns dimension of RTI in y direction. */
	virtual Eigen::MatrixXf
	getVehicleState(Eigen::Vector3f v_config);

	/** \brief Returns dimension of RTI in y direction. */
	virtual Eigen::MatrixXf
	getVehicleStateWithClearance(Eigen::Vector3f v_config);

	/** \brief Returns dimension of RTI in y direction. */
	virtual Eigen::MatrixXf
	getClearanceBoundingBoxes (Eigen::Vector3f v_config);

	virtual bool
	preProcessingRTI();

	virtual inline	 double
	pointDistFromPoint (Eigen::Vector3f point_a, Eigen::MatrixXf point_b)
	{
		Eigen::Vector3f ab = point_b - point_a;
		Eigen::Vector3f ab_sqr = ab.cwiseProduct(ab);
		float ab_dist = std::sqrt(ab_sqr.sum());
		return ab_dist;
	}

	virtual inline	 double
	pointDistFromLine (Eigen::Vector3f point_p, Eigen::MatrixXf point_a, Eigen::MatrixXf point_b)
	{
		Eigen::Vector3f ab = point_b - point_a;
		Eigen::Vector3f ap = point_p - point_a;
		float ap_proj = ap.dot(ab)/(ab.norm());
		Eigen::Vector3f ap_sqr = ap.cwiseProduct(ap);
		float ap_dist = std::sqrt(ap_sqr.sum());
		float dist = std::sqrt(ap_dist*ap_dist - ap_proj*ap_proj);

		//		Eigen::Vector3f d1=point-point1;
		//		Eigen::Vector3f d2=point-point2;
		//		Eigen::Vector3f d3=point2-point1;
		//		Eigen::Vector3f p = d1.cross(d2);
		//		return p.norm()/d3.norm();
		return dist;
	}

	virtual inline	 float
	computeConfigDistance (Eigen::Vector3f config_a, Eigen::Vector3f config_b)
	{
		Eigen::Vector3f diff = config_b - config_a;
		float dist = std::sqrt(std::pow(diff[0],2) + std::pow(diff[1],2) + 10*std::pow(diff[2]*PI_/(float)180,2));
		return dist;
	}

	//	virtual inline Eigen::MatrixXf
	//	vehicleBodyLines (Eigen::MatrixXf vehicle_state)
	//	{
	//		Eigen::VectorXf x1 = vehicle_state.row(0).leftCols(4);
	//		Eigen::Vector4f x2;
	//		x2 << x1.tail(3),x1(0);
	//		Eigen::VectorXf y1 = vehicle_state.row(1).leftCols(4);
	//		Eigen::Vector4f y2;
	//		x2 << y1.tail(3),y1(0);
	//
	//		Eigen::Vector4f x_diff = x2-x1;
	//		Eigen::Vector4f y_diff = y2-y1;
	//
	//		Eigen::MatrixXf slopes = vehicle_state.row(1).leftCols(4).rightShift(1)-vehicle_state.row(1).leftCols(4)
	//	}



protected:

	const static float PI_ = 3.14159;
	const static float MAX_PHI_ = 40;
	const static float MAX_CLEARANCE_ = 5;

	/** \brief Number of cells in x dimension. */
	int config_x_count_;

	/** \brief Number of configurations in y dimension. */
	int config_y_count_;

	/** \brief Number of configurations in y dimension. */
	int config_theta_count_;

	/** \brief Number of configurations attempted before calling it a fail. */
	int configs_limit_;

	/** \brief Number of start configurations added to the graph. */
	int start_configs_;

	/** \brief Counter that stores the number of configurations added to the tree. */
	int config_counter_;

	/** \brief Current vehicle id. */
	std::string vehicle_id_;

	/** \brief Number of configurations in y dimension. */
	float min_theta_;

	/** \brief Number of configurations in y dimension. */
	float max_theta_;

	/** \brief Vehicle velocity per second. */
	float vehicle_velocity_;

	/** \brief Change in steering angle of the vehicle per second. */
	float vehicle_steering_rate_;

	/** \brief Number of cells in x dimension. */
	float vehicle_width_;

	/** \brief Number of cells in z dimension. */
	float vehicle_length_;

	/** \brief Number of cells in z dimension. */
	float vehicle_heigt_;

	/** \brief Number of cells in z dimension. */
	float wheelbase_;

	/** \brief Number of cells in z dimension. */
	float front_track_;

	/** \brief Number of cells in z dimension. */
	float ground_clearance_;

	/** \brief Number of cells in z dimension. */
	float tire_width_;

	/** \brief Number of cells in z dimension. */
	float negative_obs_threshold_;

	/** \brief Number of cells in z dimension. */
	int max_config_neighbors_;

	/** \brief Number of cells in z dimension. */
	int sub_config_count_;

	/** \brief Goal tolerance. */
	float goal_tolerance_;

	/** \brief Goal biad probability. */
	float goal_bias_;

	/** \brief Goal tolerance. */
	float vehicle_safety_dist_;

	/** \brief Radius for config neighbors. */
	float config_neighbour_radius_;

	/** \brief Step size for sub configs between two connected configs in prm. */
	float prm_step_size_;

	/** \brief Number of cells in z dimension. */
	bool is_random_configs_;

	/** \brief Configurations import flag. */
	bool is_import_configs_;

	/** \brief Flag to show obstacles are known and imported. */
	bool is_obstacle_known_;

	/** \brief Search method that will be used for KNN. */
	OcTreePtr dem_search_;

	/** \brief Search method that will be used for neighbor configs. */
	KdTreePtr config_search_;

	/** \brief Store config cloud for neighbor search. */
	PointCloudPtr cloud_config_;

	/** \brief Matrix containing all vehicle configurations on road. */
	Eigen::MatrixXf vehicle_configs_;

	/** \brief Collision status of configurations. */
	std::vector<int> config_collision_status_;

	/** \brief Vechicle safety values. */
	std::vector<float> config_clearance_;

	/** \brief PRM edge angle ratio. */
	std::vector<float> edge_angle_ratio_;

	/** \brief Contains neighbours of each point. */
	std::vector<std::vector<int> > config_neighbours_;

	/** \brief Matrix containing vehicle state on road. */
	Eigen::MatrixXf vehicle_state_;

	/** \brief Matrix containing vehicle state on road. */
	Eigen::MatrixXf obstacles_info_;

	/** \brief Vector containing egdes of RTI graph. */
	std::vector< std::pair<int,int> > RTI_graph_;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
///** \brief This function is used as a comparator for sorting. */
//inline bool
//comparePair (std::pair<float, int> i, std::pair<float, int> j)
//{
//	return (i.first < j.first);
//}
}
//#ifdef PCL_NO_PRECOMPILE
#include <impl/rti.hpp>
//#endif
#endif /* RTI_H_ */
