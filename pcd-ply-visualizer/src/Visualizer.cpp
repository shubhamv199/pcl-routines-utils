#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
void help() {
	cout << "USAGE:\n./<executable name> <filepath>\n";
}
/**
 * -----Open 3D viewer and add point cloud-----
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}
int main(int argc, char** argv) {
	if (argc != 2) {
		help();
		exit(1);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::string file_path = argv[1];
	if (file_path.substr(file_path.find_last_of(".") + 1) == "pcd") {
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) //* load the .pcd file
				{
			cerr << "Couldn't read file :" << file_path << "\n";
			return (-1);
		}
	} else if (file_path.substr(file_path.find_last_of(".") + 1) == "ply") {
		if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) //* load the.ply file
				{
			cerr << "Couldn't read file :" << file_path << "\n";
			return (-1);
		}
	} else {
		cerr << "File format not supported (only .pcd and .ply are supported)\n";
		exit(1);
	}

	std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << (file_path.substr(file_path.find_last_of("/") + 1)) << " with the following fields: \n";

	// creates the visualization object
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(cloud);
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}
