#include "simple_hdl_viewer.h"


int main (int argc, char ** argv)
{
  std::string hdlCalibration; 
  std::string pcapFile;

  pcl::console::parse_argument (argc, argv, "-calibrationFile", hdlCalibration);
  pcl::console::parse_argument (argc, argv, "-pcapFile", pcapFile);

  pcl::HDLGrabber grabber (hdlCalibration, pcapFile);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");

  SimpleHDLViewer viewer (grabber, color_handler);
  viewer.run ();
  return (0);
}