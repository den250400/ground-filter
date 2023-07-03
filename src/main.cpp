#include "simple_hdl_viewer.h"
#include "ground_filter.h"

#include <functional>
#include <yaml-cpp/yaml.h>


int main (int argc, char ** argv)
{
  // Parse command-line arguments
  std::string hdl_calibration; 
  std::string pcap_file;
  std::string config_file;
  pcl::console::parse_argument (argc, argv, "-calibrationFile", hdl_calibration);
  pcl::console::parse_argument (argc, argv, "-pcapFile", pcap_file);
  pcl::console::parse_argument (argc, argv, "-configFile", config_file);

  // Parse ground filter settings
  YAML::Node config = YAML::LoadFile(config_file);

  // Create .pcap grabber
  pcl::HDLGrabber grabber (hdl_calibration, pcap_file);

  // Prepare the ground filter
  auto filter_fn = std::bind(GroundFilter::groundFilter, std::placeholders::_1, 
                             config["dist_thresh"].as<float>(), 
                             config["normal_max_degrees"].as<float>(), 
                             config["eps_angle"].as<float>(), 
                             config["max_iters"].as<int>());

  // Launch the visualizer
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler ("intensity");
  GroundFilter::SimpleHDLViewer viewer (grabber, color_handler, filter_fn);
  viewer.run ();

  return (0);
}