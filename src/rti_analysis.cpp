#include "include/prt.h"
#include "include/ConfigFile.h"

#include <ctime>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>

double get_cpu_time(){
	return clock() / CLOCKS_PER_SEC;
}
template <typename T>
std::string NumberToString ( T Number )
{
	std::ostringstream ss;
	ss << Number;
	return ss.str();
}

int main(int argc, char *argv[])
{
	srand(time(0));
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	std::clock_t    start;

	/*
	 * Read paths from settings.config file and vehicle count from vehicle_parameters.config
	 */
	ConfigFile cf("../configs/settings.config");
	char result[ PATH_MAX ];
		ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
		std::cout << std::string( result, (count > 0) ? count : 0 ) << std::endl;
	ConfigFile vcf("../configs/vehicle_parameters.config");
	int vehicle_count = vcf.Value("vehicle_info","vehicle_count");

	std::cout << "vehicle_count\t" << vehicle_count << std::endl;

for (int perc = 100; perc >= 10; perc-=10)
{
  for (int conf_id = 1; conf_id <= 25; conf_id++)
  {
	for (int v_idx = 2; v_idx <= vehicle_count; v_idx++)
	{
		std::string vehicle_id = "vehicle" + NumberToString(v_idx);
		std::string pointcloudFolder = (std::string)cf.Value("paths","pointcloud_folder");
		std::string configFolder = (std::string)cf.Value("paths","configuations_folder")  + vehicle_id + "/";
		std::string adjFolder = (std::string)cf.Value("paths","adjaceny_folder")  + vehicle_id + "/";
		std::string fileStartName = ".pcd";

		std::cout << vehicle_id << "\n" << pointcloudFolder << "\n" << configFolder << "\n" << adjFolder << "\n";

		//    std::string pointcloudFolder = "/home/khan/phd_ws/traversability/pointclouds/rsi/";
		//    std::string configFolder = "/home/khan/phd_ws/traversability/configs/rsi/rand_01/";
		//    std::string adjFolder = "/home/khan/phd_ws/traversability/adjacency/rsi/rand_01/";
		//    std::string fileStartName = ".pcd";

		//	std::string pointcloudFolder = "/home/wazir/phd_ws/traversability/pointclouds/tmp/";
		//	std::string configFolder = "/home/wazir/phd_ws/traversability/configs/tmp/";
		//	std::string adjFolder = "/home/wazir/phd_ws/traversability/adjacency/tmp/";
		//	std::string fileStartName = "clear2.pcd";

		DIR *dir;
		struct dirent *ent;
		if ((dir = opendir (pointcloudFolder.c_str())) != NULL) {
			while ((ent = readdir (dir)) != NULL) {
				std::string fName = ent->d_name;
				if(fName.find(fileStartName)!=std::string::npos)
				{
					bool fileProcessed = false;
					DIR *dirPRM;
					struct dirent *entPRM;
					if ((dirPRM = opendir (adjFolder.c_str())) != NULL) {
						while ((entPRM = readdir (dirPRM)) != NULL) {
							std::string fNamePRM = entPRM->d_name;
							if(fNamePRM.find("_adj")!=std::string::npos) {
								std::string fProcessed = fNamePRM.substr(0,fNamePRM.find("_adj"));
								std::string f2Process = fName.substr(0,fName.find(".pcd"));
								if (f2Process.compare(fProcessed)==0) {
									fileProcessed = true;
									std::cout <<"file processed already"<<f2Process<<"\t"<<fProcessed<<std::endl;
									break;
								}
							}
						}
					}

					if(fileProcessed) {
						continue;
					}

					//				double cpu1  = get_cpu_time();
					std::cout << endl<<"reading pointcloud file -------------------------------------\t"<<fName.c_str() << std::endl;
					std::stringstream readFile;
					readFile << pointcloudFolder.c_str() <<fName.substr(0,fName.find(".pcd")) << ".pcd";
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
					reader.read (readFile.str(), *cloud_in);

					std::cout << "Points in cloud before filtering: " << cloud_in->points.size() << std::endl;

					start = std::clock();

					/*
					 * reading obstacle file
					 */
					Eigen::MatrixXf obstacles_info_tmp = Eigen::MatrixXf::Constant(1000,4,-1000);
					Eigen::MatrixXf obstacles_info;
					std::stringstream readObstaclesInfoFile;
					readObstaclesInfoFile << pointcloudFolder.c_str() << fName.substr(0,fName.find(".pcd")) << "_obs";
					std::cout <<"file name:\t" << readObstaclesInfoFile.str() << std::endl;
					std::ifstream obsFile (readObstaclesInfoFile.str().c_str());
					if (obsFile.is_open())
					{

						//cout<< "writing data to road_"<<fileIdx<<endl;
						int c_id = 0;
						while (!obsFile.eof())
						{
							std::string line;
							std::getline (obsFile,line);
							if(line.size()==0)
							{
								break;
							}
							std::size_t nospace1 = line.find_first_not_of(" ");
							std::size_t space1 = line.find(" ",nospace1);
							std::size_t nospace2 = line.find_first_not_of(" ",space1);
							std::size_t space2 = line.find(" ",nospace2);
							std::size_t nospace3 = line.find_first_not_of(" ",space2);
							std::size_t space3 = line.find(" ",nospace3);
							std::size_t nospace4 = line.find_first_not_of(" ",space3);
							std::size_t space4 = line.find(" ",nospace4);

							std::string xStr = line.substr(nospace1,space1-nospace1);
	            std::string yStr = line.substr(nospace2,space2-nospace2);
	            std::string rStr = line.substr(nospace3,space3-nospace3);
	            std::string sStr = line.substr(nospace4,space4-nospace4);
							float xVal = std::atof(xStr.c_str());
							float yVal = std::atof(yStr.c_str());
							float rVal = std::atof(rStr.c_str());
							float sVal = std::atof(sStr.c_str());
							// std::cout <<c_id << ":\t line:	" << line <<"\t," << space1 <<"," << space2 <<"," << space3 << std::endl;

							obstacles_info_tmp.row(c_id) = Eigen::Vector4f(xVal,yVal,rVal,sVal);
							c_id++;
							//std::cout <<c_id << ":\t" << xVal <<"," << yVal <<"," << rVal <<"," << sVal << std::endl;

						}
						obsFile.close();
						obstacles_info = obstacles_info_tmp.topRows(c_id);
					}
					std::cout << obstacles_info.rows() <<" obstacles found.\n" << obstacles_info << std::endl;

					// Create a set of planar coefficients with X=Y=0,Z=1
					pcl::ModelCoefficients::Ptr proj_plane_coefficients (new pcl::ModelCoefficients ());
					proj_plane_coefficients->values.resize (4);
					proj_plane_coefficients->values[0] = proj_plane_coefficients->values[1] = 0;
					proj_plane_coefficients->values[2] = 1.0;
					proj_plane_coefficients->values[3] = 0;

					pcl::PRT<pcl::PointXYZ> RTIObj;
					RTIObj.setVehicleId(vehicle_id);
					RTIObj.setInputCloud(cloud_in);
					RTIObj.SetProjectedPlaneCoefficients(proj_plane_coefficients);
					RTIObj.setRandomConfigsFlag(true);
					RTIObj.setObstaclesInfo(obstacles_info);
          RTIObj.setConfigPercentage(perc);
					RTIObj.computeRTI();
					std::vector< std::pair<int,int> > RTI_graph;
					RTIObj.getRTIAdjacencyList(RTI_graph);
					std::vector<float> prm_edge_angel_ratios;
					RTIObj.getEdgeAngelRatios(prm_edge_angel_ratios);

					std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

					/*
					 * save adjacency
					 */
					std::stringstream saveAdj;
					saveAdj << adjFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_" << conf_id << "_" << perc <<"_adj";
					std::ofstream saveAdjFile(saveAdj.str().c_str());
					if (saveAdjFile.is_open())
					{
						for (int i=0; i < RTI_graph.size(); i++)
						{
							saveAdjFile << RTI_graph[i].first <<" "<< RTI_graph[i].second <<" "<< 1 << " " << prm_edge_angel_ratios[i] << "\n";
						}
						saveAdjFile.close();
					}else {
						std::cout<<"Error: can not find directory"<<std::endl;
						exit(0);
					}

					/*
					 * save configurations
					 */
					Eigen::MatrixXf vehicle_configs = RTIObj.getConfigurations();
					//				int config_counter = RTIObj.getConfigCounter();
					//				std::cout << "config counter:" << config_counter << std::endl;
					std::vector<int> config_validity_status = RTIObj.getConfigurationsValidityStatus();
					std::vector<float> config_clearance = RTIObj.getConfigurationsClearance();

					std::stringstream saveRTI;
					saveRTI << configFolder.c_str() << fName.substr(0,fName.find(".pcd")) <<"_" << conf_id << "_" << perc<<"_conf";
					std::ofstream saveRTIFile(saveRTI.str().c_str());
					if (saveRTIFile.is_open())
					{
						for (int i=0; i < vehicle_configs.rows(); i++)
						{
							saveRTIFile << vehicle_configs.row(i) <<" "<<config_validity_status[i]<< " " << config_clearance[i] << "\n";
						}
						saveRTIFile.close();
					}else {
						std::cout<<"Error: can not find directory"<<std::endl;
						exit(0);
					}
					//				double cpu2  = get_cpu_time();
					//				std::cout << "CPU Time after prm complete = " << cpu2  - cpu1  << std::endl;
					std::cout << "file written successfully" << endl<< std::endl;

					//  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = RTIObj.getDEMVisibilityCloud ();
					//  writer.write<pcl::PointXYZRGB> ("road_rgb.pcd", *colored_cloud, false);
					//				boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;
					//				mesh_ptr_->polygons[0].vertices[0] = 0;
					//				mesh_ptr_->polygons[0].vertices[1] = 0;
					//				mesh_ptr_->polygons[0].vertices[2] = 0;
					//				mesh_ptr_->polygons[1].vertices[0] = 1;
					//				mesh_ptr_->polygons[1].vertices[1] = 0;
					//				mesh_ptr_->polygons[1].vertices[2] = 0;
					//				mesh_ptr_->polygons[2].vertices[0] = 1;
					//				mesh_ptr_->polygons[2].vertices[1] = 2;
					//				mesh_ptr_->polygons[2].vertices[2] = 0;

					//				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
					//				viewer->setBackgroundColor (0, 0, 0);
					//				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
					//				viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud, rgb, "sample cloud");
					//				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
					//				viewer->addCoordinateSystem (1.0);
					//				viewer->initCameraParameters ();

					//				pcl::visualization::CloudViewer viewer ("Cluster viewer");

					//				viewer.addPolygonMesh	(mesh_ptr_,"polygon",0 );
					//				viewer.showCloud(colored_cloud);
					//				while (!viewer.wasStopped ())
					//				{
					//				}

					//				while (!viewer->wasStopped ())
					//				{
					//					viewer->spinOnce (100);
					//					boost::this_thread::sleep (boost::posix_time::microseconds (100000));
					//				}
				}
			}
			closedir (dir);
		} else {
			perror ("could not open directory");
			return EXIT_FAILURE;
		}
	}
}
}
	return 0;
}
