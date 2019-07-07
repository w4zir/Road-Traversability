#include <ctime>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char *argv[])
{
	srand ((unsigned int) time (NULL));
	if (argc < 9)
	{
		cerr << "usage: " << argv[0] << "input_file, output_file, x y z w l h points theta vx vy vz." << endl;
		exit (EXIT_FAILURE);
	}
	std::string inFile = argv[1];
	std::string opFile = argv[2];
	float rWid;
	float rLen;
	float rHeit;
	int points_count;
	float theta;
	float vx;
	float vy;
	float vz;
	float tx;
	float ty;
	float tz;
std::istringstream (argv[3]) >> rWid;
std::istringstream (argv[4]) >> rLen;
std::istringstream (argv[5]) >> rHeit;
std::istringstream (argv[6]) >> points_count;
std::istringstream (argv[7]) >> theta;
std::istringstream (argv[8]) >> vx;
std::istringstream (argv[9]) >> vy;
std::istringstream (argv[10]) >> vz;
std::istringstream (argv[11]) >> tx;
std::istringstream (argv[12]) >> ty;
std::istringstream (argv[13]) >> tz;

	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_op (new pcl::PointCloud<pcl::PointXYZRGB>);

	std::cout << "Starting process." 	<< std::endl;

	std::stringstream inStream;
	inStream << inFile.c_str();
	reader.read (inStream.str(), *cloud_in);
	std::cout << "Points in cloud before obstacle addition: " << cloud_in->points.size() << std::endl;

    // for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud_in->points.begin(); it!= cloud_in->points.end(); it++){
    // it->r = 0;
		// it->g = 255;
		// it->b = 0;
    // }
	// // Generate pointcloud data
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //  cloud->width = cSize;
  //  cloud->height = 1;
  //  cloud->points.resize (cloud->width * cloud->height);
	//
  //  for (size_t i = 0; i < cloud->points.size (); ++i)
  //  {
	//
  //    //cloud->points[i].b = 255;
  //  }

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

   // Define a translation of 2.5 meters on the x axis.
  transform.translation() << tx, ty, tz;

   // The same rotation matrix as before; theta radians arround Z axis
  // transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
	transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f(vx,vy,vz)));

	for (int i=0; i< points_count; i++)
	{
		pcl::PointXYZRGB point;
		point.x = rWid * rand () / (RAND_MAX + 1.0f) - rWid/2;
		point.y = rLen * rand () / (RAND_MAX + 1.0f) - rLen/2;
		point.z = rHeit * rand () / (RAND_MAX + 1.0f) - rHeit/2;
		point.r = 255;
		point.g = 0;
		point.b = 0;

		tmp->push_back(point);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*tmp, *transformed_cloud, transform);

	cloud_op = cloud_in;
	*cloud_op += *transformed_cloud;

	std::stringstream opStream;
	opStream << "/home/az/git_repos/phd/road-traversability/data/pointclouds/"<<opFile.c_str();
	writer.write<pcl::PointXYZRGB> (opStream.str(), *cloud_op, false);

	//	pcl::PointCloud<pcl::PointXYZ>::Ptr dem_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//	pcl::DEM<pcl::PointXYZ> obj;
	//
	//	obj.setInputCloud(cloud_in);
	//	//seg.setRoadPlaneCoefficients(model_coeff_pass);
	//	//seg.setRoadNormal(Eigen::Vector3f(model_coeff_pass->values[0],model_coeff_pass->values[1],model_coeff_pass->values[2]));
	//	obj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
	//	obj.computeDEM();
	//	obj.getDEMClusters(dem_clusters);
	//	prmObj.getDEMCloud(*dem_cloud);

	//	std::cout << "clusters size" << dem_clusters.size() << std::endl;

	//	pcl::PointCloud <pcl::PointXYZRGB>::Ptr prm_cloud = prmObj.getPRMVisibilityCloud ();
	//	prm_cloud->height = 1;
	//	prm_cloud->width = prm_cloud->points.size();
	//	writer.write<pcl::PointXYZRGB> ("../data/prm_cloud.pcd", *prm_cloud, false);
	//
	//	pcl::PointCloud <pcl::PointXYZRGB>::Ptr dem_cloud = prmObj.getDEMVisibilityCloud ();
	//	dem_cloud->height = 1;
	//	dem_cloud->width = dem_cloud->points.size();
	//	writer.write<pcl::PointXYZRGB> ("../data/dem_cloud.pcd", *dem_cloud, false);
	//

	/*
	 * visualize output cloud
	 */
	//	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	//
	//	viewer.showCloud(cloud_op);
	//	while (!viewer.wasStopped ())
	//	{
	//	}

	return 0;
}
