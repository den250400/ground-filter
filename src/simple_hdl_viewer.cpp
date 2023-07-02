#include "simple_hdl_viewer.h"

using namespace std::chrono_literals;
using namespace pcl::console;
using namespace pcl::visualization;

SimpleHDLViewer::SimpleHDLViewer(pcl::Grabber& grabber,
                                 pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>& handler) :
    cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL HDL Cloud")),
    grabber_(grabber),
    handler_(handler)
{
}

void SimpleHDLViewer::cloud_callback(const CloudConstPtr& cloud)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  cloud_ = cloud;
}

void SimpleHDLViewer::run()
{
  cloud_viewer_->addCoordinateSystem(3.0);
  cloud_viewer_->setBackgroundColor(0, 0, 0);
  cloud_viewer_->initCameraParameters();
  cloud_viewer_->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
  cloud_viewer_->setCameraClipDistances(0.0, 50.0);

  std::function<void(const CloudConstPtr&)> cloud_cb =
      [this](const CloudConstPtr& cloud) { cloud_callback(cloud); };
  boost::signals2::connection cloud_connection = grabber_.registerCallback(
      cloud_cb);

  grabber_.start();

  while (!cloud_viewer_->wasStopped())
  {
    CloudConstPtr cloud;

    // See if we can get a cloud
    if (cloud_mutex_.try_lock())
    {
      cloud_.swap(cloud);
      cloud_mutex_.unlock();
    }

    if (cloud)
    {
      handler_.setInputCloud(cloud);
      if (!cloud_viewer_->updatePointCloud(cloud, handler_, "HDL"))
        cloud_viewer_->addPointCloud(cloud, handler_, "HDL");

      cloud_viewer_->spinOnce();
    }

    if (!grabber_.isRunning())
      cloud_viewer_->spin();

    std::this_thread::sleep_for(100us);
  }

  grabber_.stop();

  cloud_connection.disconnect();
}
