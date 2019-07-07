#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

using namespace pcl;
using namespace std;


int main(int argc,char *argv[])
{
  srand ((unsigned int) time (NULL));
  // Plane coefficient
  double a;
  double b;
  double c;
  double d;

  // road dimensions
	double rLen = 20.0;
	double rWid = 6.0;

 // No of points in cloud
	double cSize = 100000;

  if (argc < 3)
  {
    cerr << "usage: " << argv[0] << " rLen rWid cSize outputFile" << endl;
    exit (EXIT_FAILURE);
  }

  /// read plane coefficient
  istringstream (argv[1]) >> rLen;
  istringstream (argv[2]) >> rWid;
  istringstream (argv[3]) >> cSize;   // threshold for DoN magnitude

  /// output file
  string outputFile = argv[4];

 // Generate pointcloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = cSize;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = rWid * rand () / (RAND_MAX + 1.0f)-rWid/2;
    cloud->points[i].y = rLen * rand () / (RAND_MAX + 1.0f)-rLen/2;
    cloud->points[i].z = 0;
    //cloud->points[i].b = 255;
  }


 pcl::PCDWriter writer;
 // writer.write ("clear0.pcd", *cloud, false);

 	std::stringstream opStream;
 	opStream << "/home/az/git_repos/phd/road-traversability/data/pointclouds/"<<outputFile.c_str();
 	writer.write<pcl::PointXYZ> (opStream.str(), *cloud, false);


return 0;
}
