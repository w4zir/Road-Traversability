/*
 * height_estimate.hpp
 *
 *  Created on: Dec 1, 2015
 *      Author: mudassir
 */

#ifndef DEM_HPP_
#define DEM_HPP_

#include "../dem.h"
#include "../ConfigFile.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::DEM<PointT>::DEM () :
debug_mode_ (false),
neighbour_radius_ (0.05),
dimension_x_ (41),
dimension_y_ (201),
road_min_x_ (-2),
road_max_x_ (2),
road_min_y_ (-10),
road_max_y_ (10),
road_width_ (4),
road_length_ (20),
distance_max_ (10),
distance_cluster_ (0.15),
is_projected_ (false),
search_ (),
cloud_project_(),
cloud_dem_ (),
point_neighbours_ (0),
dem_clusters_ (0),
plane_threshold_ (0.01),
projected_plane_coeff_ (),
road_coefficients_()
{
	//search_ =  typename pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>);
loadParameters ();
	//	std::cout << neighbour_radius_ << ", " << dimension_x_ << ", " << road_width_ << ", " << distance_max_ << ", " << distance_cluster_ << std::endl;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::DEM<PointT>::~DEM ()
{
	if (search_ != 0)
		search_.reset ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::loadParameters ()
{
	/*
	 * Read road information and dem settings from settings.config file
	 */
	std::cout << "Loading DEM parameters."<< std::endl;
	ConfigFile cf("../configs/settings.config");

	debug_mode_  = (bool)cf.Value("general","debug_mode");
  if (debug_mode_)
	{
		std::cout << "Running with debug mode on." << std::endl;
	} else
	{
		std::cout << "Running with debug mode off." << std::endl;
	}
	road_min_x_		=	cf.Value("road_info","road_min_x");
	road_max_x_		=	cf.Value("road_info","road_max_x");
	road_min_y_		=	cf.Value("road_info","road_min_y");
	road_max_y_		=	cf.Value("road_info","road_max_y");
	road_width_		=	cf.Value("road_info","road_width");
	road_length_	=	cf.Value("road_info","road_length");

	neighbour_radius_	= 	cf.Value("dem_parameters","neighbour_radius");
	dimension_x_	= 	cf.Value("dem_parameters","dimension_x");
	dimension_y_	=	cf.Value("dem_parameters","dimension_y");
	distance_max_	=	cf.Value("dem_parameters","distance_max");
	distance_cluster_	=	cf.Value("dem_parameters","distance_cluster");
	plane_threshold_	=	cf.Value("dem_parameters","plane_threshold");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::DEM<PointT>::getDimensionX () const
{
	return (dimension_x_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setDimensionX (int dim_x)
{
	dimension_x_ = dim_x;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::DEM<PointT>::getDimensionY () const
{
	return (dimension_y_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setDimensionY (int dim_y)
{
	dimension_y_ = dim_y;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::DEM<PointT>::getMaxDistanceFromRoad () const
{
	return (distance_max_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setMaxDistanceFromRoad (float max_distance)
{
	distance_max_ = max_distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::DEM<PointT>::getClusterSegregationDistance () const
{
	return (distance_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setClusterSegregationDistance (float cluster_distance)
{
	distance_cluster_ = cluster_distance;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::DEM<PointT>::getIsProjectedCloud () const
{
	return (is_projected_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setIsProjectCloud (bool is_projected)
{
	is_projected_ = is_projected;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::DEM<PointT>::ModelCoefficient
pcl::DEM<PointT>::getProjectedPlaneCoefficients () const
{
	return (projected_plane_coeff_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::SetProjectedPlaneCoefficients (const ModelCoefficient plane_coefficients)
{
	projected_plane_coeff_ = plane_coefficients;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::DEM<PointT>::ModelCoefficient
pcl::DEM<PointT>::getRoadPlaneCoefficients () const
{
	return (road_coefficients_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setRoadPlaneCoefficients (const ModelCoefficient road_coefficients)
{
	road_coefficients_ = road_coefficients;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
pcl::DEM<PointT>::getNeighboursRadius () const
{
	return (neighbour_radius_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setNeighboursRadius (float neighbour_radius)
{
	neighbour_radius_ = neighbour_radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::DEM<PointT>::KdTreePtr
pcl::DEM<PointT>::getSearchMethod () const
{
	return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setSearchMethod (const KdTreePtr& tree)
{
	if (search_ != 0)
		search_.reset ();

	search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::getProjectedCloud (PointCloudPtr &project)
{
	project = cloud_project_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::setProjectedCloud (PointCloudPtr project)
{
	cloud_project_ = project;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::getDEMClusters (std::vector < std::vector < clusters> >& dem_clusters)
{
	dem_clusters = dem_clusters_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::getDEMCloud (PointCloud &pointcloud)
{
	pointcloud = *cloud_dem_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::getMinMax()
{
	std::cout <<"min max of original pointcloud \n";
	PointT minPts,maxPts;
	pcl::getMinMax3D(*cloud_project_,minPts,maxPts);
	road_min_x_ = minPts.x;
	road_min_y_ = minPts.y;
	road_max_x_ = maxPts.x;
	road_max_y_ = maxPts.y;
	std::cout << "minX:"<<minPts.x<<" minY:"<<minPts.y<<" minZ:"<<minPts.z<<std::endl;
	std::cout << "maxX:"<<maxPts.x<<" maxY:"<<maxPts.y<<" maxZ:"<<maxPts.z<<std::endl;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::transformCloud2XYPlane()
{
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (plane_threshold_);
	seg.setInputCloud (input_);
	seg.segment (*inliers, *coefficients);
	float pCoeffMod = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
	std::cout << "coefficients of plane are: "<< *coefficients << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr xyTransformCloud (new pcl::PointCloud<pcl::PointXYZ> ());
	//get transformation
	Eigen::Vector3f normalXYPlane (0.0,0.0,1.0);
	Eigen::Vector3f normalRoadPlane (coefficients->values[0]+0.0000000001,coefficients->values[1]+0.0000000001,coefficients->values[2]+0.0000000001);
	Eigen::Vector3f cNormal = normalRoadPlane.cross(normalXYPlane);
	float sNormal = sqrt(cNormal.squaredNorm());
	float dNormal = normalRoadPlane.dot(normalXYPlane);
	Eigen::Matrix3f cNormalSkewSymm;
	cNormalSkewSymm << 0, -cNormal(2), cNormal(1), cNormal(2), 0, -cNormal(0), -cNormal(1), cNormal(0), 0;
	Eigen::Matrix3f rot2XYPlane;
	rot2XYPlane = Eigen::Matrix3f::Identity(3,3)+cNormalSkewSymm+cNormalSkewSymm*cNormalSkewSymm*((1-dNormal)/pow(sNormal,2));
	/*  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block(0,0,3,3) =  rot2XYPlane;*/
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.0, 0.0,coefficients->values[3];
	transform.rotate (rot2XYPlane);
	pcl::transformPointCloud (*input_, *xyTransformCloud, transform);
	input_ = xyTransformCloud;
	std::cout <<"points in transformed pointcloud:\t"<<input_->points.size() << std::endl;
// pcl::PCDWriter writer;
// writer.write<pcl::PointXYZ> ("transformed.pcd", *input_, false);
	// for kitti dataset we have to swap x and y coordinates
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::projectPointCloud ()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_project (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (input_);
	proj.setModelCoefficients (projected_plane_coeff_);
	proj.filter (*cloud_project);
	cloud_project_ = cloud_project;
	std::cout <<"pointcloud projected." << std::endl;
// pcl::PCDWriter writer;
// 	writer.write<pcl::PointXYZ> ("cloud_project.pcd", *cloud_project_, false);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::computeDEM ()
{
	bool computation_is_possible = initCompute ();
	if ( !computation_is_possible )
	{
		deinitCompute ();
		return;
	}

	// pre-process i.e. project cloud if not already projected and generate empty DEM
	bool is_processed = preProcessing();
	if(!is_processed)
	{
		std::cout<<"Error, could not pre-process cloud." << std::endl;
		return;
	}
	std::cout << "pre-processing for DEM done." << std::endl;

	// find points within each grid cell of DEM
	findPointNeighbours ();
	std::cout << "neighbors done." << std::endl;

	int point_numbers = cloud_dem_->points.size();
	std::cout << "indices size:" << point_numbers << std::endl;

	//	std::cout << "projection done." << std::endl;


	double coeff_sqrt = std::sqrt(std::pow(projected_plane_coeff_->values[0],2)+std::pow(projected_plane_coeff_->values[1],2)+std::pow(projected_plane_coeff_->values[2],2));


	for(int i_point=0; i_point < point_numbers; i_point++)
	{
		int neighbor_count = point_neighbours_[i_point].size();
		if(neighbor_count<=0)
		{
			continue;
		}
		//std::cout<<"point \t" << i_point << "processing with neighbor:" << neighbor_count << std::endl;
		std::vector< std::pair<float, int> > point_distances;
		std::pair<float, int> pair;
		point_distances.resize (neighbor_count, pair);
		for (int n_point=0; n_point < neighbor_count ; n_point++)
		{
			int neighbor_indx = point_neighbours_[i_point][n_point];
			//std::cout<<"neighbor # \t" << n_point << " with index:" << neighbor_indx << std::endl;
			double dist_from_road = (projected_plane_coeff_->values[0]*input_->points[neighbor_indx].x +
					projected_plane_coeff_->values[1]*input_->points[neighbor_indx].y +
					projected_plane_coeff_->values[2]*input_->points[neighbor_indx].z + projected_plane_coeff_->values[3])/
							(coeff_sqrt);
			point_distances[n_point].first = dist_from_road;
			point_distances[n_point].second = neighbor_indx;
		}
		//std::cout << "calling computeCellClusters." << std::endl;
		computeCellClusters(point_distances,i_point);
	}
	std::cout << "DEM computation done im DEM.hpp." << std::endl;

	//	output = *height_cloud;
	deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::computeCellClusters (std::vector< std::pair<float, int> > p_dist, int p_indx)
{
	std::vector< clusters > cluster_limits;
	dem_clusters_.resize(cloud_dem_->points.size(), cluster_limits);
	std::sort (p_dist.begin (), p_dist.end (), comparePair);
	float cluster_min = p_dist[0].first;
	float cluster_max = p_dist[0].first;
	int cluster_size = 1;
	for(int i=1; i < p_dist.size(); i++)
	{
		float distance_of_points = p_dist[i].first-p_dist[i-1].first;
		if(distance_of_points>distance_cluster_)
		{
			clusters cls_info = {cluster_min,cluster_max,cluster_size};
			cluster_limits.push_back(cls_info);
			//			cluster_limits.push_back(std::make_tuple(cluster_min,cluster_max,cluster_size));
			cluster_min = p_dist[i].first;
			cluster_max = p_dist[i].first;
			cluster_size = 1;
		} else {
			cluster_max = p_dist[i].first;
			cluster_size++;
		}
		if(i ==( p_dist.size()-1))
		{
			//			cluster_limits.push_back(std::make_tuple(cluster_min,cluster_max,cluster_size));
			clusters cls_info = {cluster_min,cluster_max,cluster_size};
			cluster_limits.push_back(cls_info);
		}
	}
	//	std::cout << "cluster size:" << cluster_limits.size() << std::endl;
	dem_clusters_[p_indx].swap (cluster_limits);
	//	return cluster_limits;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::findPointNeighbours ()
{
	int point_number = static_cast<int> (cloud_dem_->points.size ());
	std::vector<int> neighbours;
	std::vector<float> distances;

	int n_zero = 0;
	//std::cout << "cloud_dem_->points.size ():" << cloud_dem_->points.size () << std::endl;
	point_neighbours_.resize (point_number, neighbours);
	for (int i_point = 0; i_point < point_number; i_point++)
	{
		neighbours.clear ();
		pcl::PointXYZ point;
		point.x = cloud_dem_->points[i_point].x;
		point.y = cloud_dem_->points[i_point].y;
		point.z = cloud_dem_->points[i_point].z;
		search_ ->radiusSearch (point, neighbour_radius_, neighbours, distances);
		if(neighbours.size()<=1)
		{
			n_zero++;
		}
	//	std::cout << "total:"<< point_number <<"\t id:"<< i_point << "\t neighbours:" << neighbours.size() << std::endl;
		point_neighbours_[i_point].swap (neighbours);
	}
	std::cout <<"zero neighbor count:" << n_zero << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::DEM<PointT>::preProcessing ()
{
	std::cout << "Pre-processing for DEM." << std::endl;

	// load parameters
	// loadParameters ();
	std::cout << "road_min_x:" << road_min_x_ << "\t road max_x:" << road_max_x_<< std::endl;
	std::cout << "road_min_y:" << road_min_y_ << "\t road max_y:" << road_max_y_<< std::endl;

	// if user forgot to pass point cloud or if it is empty
	if ( input_->points.size () == 0 )
		return (false);

	// transform pointcloud to xy-plane
	transformCloud2XYPlane();

	// check whether input cloud is already projected
	if(!is_projected_)
	{
		projectPointCloud();
	}

	// get min and max of point cloud
	getMinMax();

	// if user forgot to pass normals or the sizes of point and normal cloud are different
	if ( cloud_project_ == 0 || input_->points.size () != cloud_project_->points.size () )
		return (false);

	// if user didn't set search method
	if (!search_)
		search_.reset (new pcl::search::KdTree<PointT>);

	//search_->setInputCloud (cloud_project_);
	search_->setInputCloud(cloud_project_);
	//setProjectedCloud(cloud_project_);
	generateEmptyDEM();
	return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::DEM<PointT>::generateEmptyDEM ()
{
	std::cout << "Generating empty DEM. \n";
	pcl::PointCloud<PointXYZ>::Ptr dem_cloud (new pcl::PointCloud<PointXYZ> ());

	double x_resolution = (road_width_ /(double)(dimension_x_ + 1));
	double y_resolution = (road_length_ /(double)(dimension_y_ + 1));

	for (int i=0;i<dimension_y_;i++) {
		for (int j=0;j<dimension_x_;j++) {
			PointXYZ point;
			point.x = -(double)road_width_/2 + j*x_resolution;
			point.y = -(double)road_length_/2 + + i*y_resolution;
			point.z = 0;
			dem_cloud->points.push_back(point);
		}
	}
	cloud_dem_ = dem_cloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::DEM<PointT>::getDEMVisibilityCloud ()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
	colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

	for (int i=0; i < dem_clusters_.size(); i++)
	{
		for (int j=0; j < dem_clusters_[i].size(); j++)
		{
			//			float min_dist;
			//			float max_dist;
			//			int cluster_size;
			//			std::tie(min_dist, max_dist, cluster_size) = dem_clusters_[i][j];
			float min_dist = dem_clusters_[i][j].min_dist;
			float max_dist = dem_clusters_[i][j].max_dist;
			int cluster_size = dem_clusters_[i][j].cluster_size;
			//			if(min_dist!=0 || max_dist!=0)
			//				std::cout<<"cluster size:" << dem_clusters_[i].size() << "\t min:"<< min_dist <<"\t max:" << max_dist<< std::endl;
			int i_point = i; //dimension_x_*j+i;
			//			if(min_dist == max_dist)
			if(min_dist < 0)
			{
				pcl::PointXYZRGB point;
				point.x = cloud_dem_->points[i].x;// *(cloud_dem_->points[i_point].data);
				point.y = cloud_dem_->points[i].y;//*(cloud_dem_->points[i_point].data + 1);
				point.z = min_dist;
				point.r = 0;
				point.g = 0;
				point.b = 255;
				colored_cloud->points.push_back (point);
			} else if (min_dist == 0)
			{
				pcl::PointXYZRGB point;
				point.x = cloud_dem_->points[i].x;// *(cloud_dem_->points[i_point].data);
				point.y = cloud_dem_->points[i].y;//*(cloud_dem_->points[i_point].data + 1);
				point.z = min_dist;
				point.r = 0;
				point.g = 255;
				point.b = 0;
				colored_cloud->points.push_back (point);
			} else
			{
				pcl::PointXYZRGB point;
				point.x = cloud_dem_->points[i].x;// *(cloud_dem_->points[i_point].data);
				point.y = cloud_dem_->points[i].y;//*(cloud_dem_->points[i_point].data + 1);
				point.z = min_dist;
				point.r = 255;
				point.g = 0;
				point.b = 0;
				colored_cloud->points.push_back (point);
			}

			if(max_dist < 0)
			{
				pcl::PointXYZRGB point;
				point.x = cloud_dem_->points[i].x;// *(cloud_dem_->points[i_point].data);
				point.y = cloud_dem_->points[i].y;//*(cloud_dem_->points[i_point].data + 1);
				point.z = max_dist;
				point.r = 0;
				point.g = 0;
				point.b = 255;
				colored_cloud->points.push_back (point);
			} else if (min_dist == 0)
			{
				pcl::PointXYZRGB point;
				point.x = cloud_dem_->points[i].x;// *(cloud_dem_->points[i_point].data);
				point.y = cloud_dem_->points[i].y;//*(cloud_dem_->points[i_point].data + 1);
				point.z = max_dist;
				point.r = 0;
				point.g = 255;
				point.b = 0;
				colored_cloud->points.push_back (point);
			} else
			{
				pcl::PointXYZRGB point;
				point.x = cloud_dem_->points[i].x;// *(cloud_dem_->points[i_point].data);
				point.y = cloud_dem_->points[i].y;//*(cloud_dem_->points[i_point].data + 1);
				point.z = max_dist;
				point.r = 255;
				point.g = 0;
				point.b = 0;
				colored_cloud->points.push_back (point);
			}

			//
			//				pcl::PointXYZRGB point1;
			//				point1.x = cloud_dem_->points[i].x;// *(cloud_dem_->points[i_point].data);
			//				point1.y = cloud_dem_->points[i].y;//*(cloud_dem_->points[i_point].data + 1);
			//				point1.z = min_dist;
			//				point1.r = 0;
			//				point1.g = 0;
			//				point1.b = 255;
			//				colored_cloud->points.push_back (point1);
			//
			//				pcl::PointXYZRGB point2;
			//				point2.x = cloud_dem_->points[i].x;// *(cloud_dem_->points[i_point].data);
			//				point2.y = cloud_dem_->points[i].y;//*(cloud_dem_->points[i_point].data + 1);
			//				point2.z = max_dist;
			//				point2.r = 255;
			//				point2.g = 0;
			//				point2.b = 0;
			//				colored_cloud->points.push_back (point2);
			//		}
		}
	}

	return (colored_cloud);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
//pcl::HeightEstimate<PointT>::getColoredCloud ()
//{
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
//
//  if (!point_labels_.empty ())
//  {
//    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
//
//    colored_cloud->width = input_->width;
//    colored_cloud->height = input_->height;
//    colored_cloud->is_dense = input_->is_dense;
//    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
//    {
//      pcl::PointXYZRGB point;
//      point.x = *(input_->points[i_point].data);
//      point.y = *(input_->points[i_point].data + 1);
//      point.z = *(input_->points[i_point].data + 2);
//      point.r = 0;
//      point.g = 0;
//      point.b = 0;
//      if(point_labels_[i_point] == 1)
//      {
//        point.g = 255;
//      }else
//      {
//        point.r = 255;
//      }
//      colored_cloud->points.push_back (point);
//    }
//  }
//
//  return (colored_cloud);
//}

#define PCL_INSTANTIATE_DEM(T) template class pcl::DEM<T>;

#endif /* DEM_HPP_ */
