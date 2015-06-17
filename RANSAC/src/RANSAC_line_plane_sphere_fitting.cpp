#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
typedef pcl::PointXYZ PointT;

/**
 * -----Open 3D viewer and add point cloud-----
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters();
	return (viewer);
}
void help(){
	cout<<"USAGE:\n./<executable name> <filepath> <-l/-f/-sf\n";
}
int main(int argc, char** argv) {
	//check if number of arguments is proper
	if(argc!=3){
		help();
		exit(0);
	}

	// initialize PointClouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) //* load the file
			{
		cerr << "Couldn't read file :" << argv[1] << "\n";
		return (-1);
	}

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

	//RANSAC for Line model
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<PointT>());
	if (pcl::console::find_argument(argc, argv, "-l") >= 0) {
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_l(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.05);

		seg.setInputCloud(cloud);
		seg.segment(*inliers_l, *coefficients);
		// Write the line inliers to disk
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers_l);
		extract.setNegative(false);

		extract.filter(*cloud_line);
	}


	if (pcl::console::find_argument(argc, argv, "-f") >= 0) {
		//RANSAC for Plane model
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	} else if (pcl::console::find_argument(argc, argv, "-sf") >= 0) {
		//RANSAC for Sphere model
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_s);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.getInliers(inliers);
	}

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

	// creates the visualization object and adds either our orignial cloud or all of the inliers
	// depending on the command line arguments specified.
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0) {
		viewer = simpleVis(final);
	} else if (pcl::console::find_argument(argc, argv, "-l") >= 0) {
		viewer = simpleVis(cloud_line);
	} else
		viewer = simpleVis(cloud);
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
