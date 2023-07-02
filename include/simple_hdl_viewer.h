#pragma once

#include <chrono>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

class SimpleHDLViewer
{
public:
  typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
  typedef Cloud::ConstPtr CloudConstPtr;

  SimpleHDLViewer(pcl::Grabber& grabber,
                  pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>& handler);

  void run();

private:
  void cloud_callback(const CloudConstPtr& cloud);

  pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;
  pcl::Grabber& grabber_;
  std::mutex cloud_mutex_;
  CloudConstPtr cloud_;
  pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>& handler_;
};
