#pragma once

#include <chrono>
#include <mutex>
#include <functional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

namespace GroundFilter
{
class SimpleHDLViewer
{
public:
  typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
  typedef Cloud::Ptr CloudPtr;
  typedef Cloud::ConstPtr CloudConstPtr;

  SimpleHDLViewer(pcl::Grabber& grabber,
                  pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>& handler,
                  std::function<void(CloudPtr)> filter_function = [](CloudPtr cloud){});

  void run();

private:
  void cloudCallback(const CloudConstPtr& cloud);

  pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;
  pcl::Grabber& grabber_;
  std::mutex cloud_mutex_;
  CloudPtr cloud_;
  pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>& handler_;
  std::function<void(CloudPtr)> filter_function_;
};
}