#define NOMINMAX
#include <iostream>
#include <string>
#include "benewake_lidar_driver.h"
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "benewake_point.h"

int main()
{
	std::string file_in = "D:/BenewakeWorkspace/X-project/data/PCD/11_02_17_751.pcd";					// path of file need to be converted
	std::string file_out = "D:/BenewakeWorkspace/X-project/data/PCD/11_02_17_751_converted.pcd";		// path of file converted
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = std::make_shared<pcl::visualization::PCLVisualizer>();
	viewer->addCoordinateSystem(2.0);
	viewer->setCameraPosition(-40.0, 0.0, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	bwPCLPointCloud bw_pcl_pc;
	pcl::io::loadPCDFile(file_in, bw_pcl_pc);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	cloud->resize(bw_pcl_pc.size());
	for (int i = 0; i < bw_pcl_pc.size(); ++i)
	{
		if (i % 100 == 0)
			std::cout << bw_pcl_pc.points[i].x << " " << bw_pcl_pc.points[i].y << " " << bw_pcl_pc.points[i].z << " "
			<< bw_pcl_pc.points[i].intensity << " " << bw_pcl_pc.points[i].row << " " << (int)bw_pcl_pc.points[i].channel << " " << (int)bw_pcl_pc.points[i].confidence << std::endl;
		cloud->points[i].x = bw_pcl_pc.points[i].x;
		cloud->points[i].y = bw_pcl_pc.points[i].y;
		cloud->points[i].z = bw_pcl_pc.points[i].z;
		cloud->points[i].intensity = bw_pcl_pc.points[i].intensity;
	}
	pcl::io::savePCDFileBinary(file_out, *cloud);

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color(cloud, "intensity");
	viewer->addPointCloud(cloud, color, "cloud");
	viewer->spin();
}
