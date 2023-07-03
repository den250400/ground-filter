#include "simple_hdl_viewer.h"

using namespace std::chrono_literals;
using namespace pcl::console;
using namespace pcl::visualization;

GroundFilter::SimpleHDLViewer::SimpleHDLViewer(pcl::Grabber& grabber,
                                 pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>& handler,
                                 std::function<void(CloudPtr)> filter_function) :
    cloud_viewer_(new pcl::visualization::PCLVisualizer("PCL cloud viewer")),
    grabber_(grabber),
    handler_(handler),
    cloud_(new Cloud),
    filter_function_(filter_function)
{
}

void GroundFilter::SimpleHDLViewer::cloudCallback(const CloudConstPtr& cloud)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  
  cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*cloud, *cloud_);
  filter_function_(cloud_);
}

void GroundFilter::SimpleHDLViewer::run()
{
  cloud_viewer_->addCoordinateSystem(3.0);
  cloud_viewer_->setBackgroundColor(0, 0, 0);
  cloud_viewer_->initCameraParameters();
  cloud_viewer_->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
  cloud_viewer_->setCameraClipDistances(0.0, 50.0);

  std::function<void(const CloudConstPtr&)> cloud_cb =
      [this](const CloudConstPtr& cloud) { cloudCallback(cloud); };
  boost::signals2::connection cloud_connection = grabber_.registerCallback(
      cloud_cb);

  grabber_.start();

  while (!cloud_viewer_->wasStopped())
  {
    CloudPtr cloud;

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
