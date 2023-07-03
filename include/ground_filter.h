// PCL
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>


namespace GroundFilter
{
void filterPointCloudByNormalAngle(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                   const pcl::Normal& target_normal,
                                   double max_degrees,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud)
{
    // Create a normal estimation object and compute normals
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(30);  // Number of neighbors to consider for normal estimation
    ne.compute(*normals);

    // Filter points based on normal angle
    filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        const pcl::Normal& normal = normals->at(i);
        double dot_product = normal.normal_x * target_normal.normal_x +
                             normal.normal_y * target_normal.normal_y +
                             normal.normal_z * target_normal.normal_z;
        double angle = acos(dot_product) * 180.0 / M_PI;  // Convert to degrees

        if (angle <= max_degrees)
            filtered_cloud->push_back(cloud->at(i));
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr filterPointCloudByDistance(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::vector<float>& plane_coefficients, float dist_threshold)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Extract plane coefficients
    float a = plane_coefficients[0];
    float b = plane_coefficients[1];
    float c = plane_coefficients[2];
    float d = plane_coefficients[3];

    for (const auto& point : cloud->points)
    {
        // Calculate the distance from the point to the plane
        float distance = pcl::pointToPlaneDistanceSigned(point, a, b, c, d);

        // Check if the point is farther than the threshold from the plane
        if (std::abs(distance) > dist_threshold)
        {
            // Add the point to the filtered cloud
            filteredCloud->push_back(point);
        }
    }

    return filteredCloud;
}

void groundFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float dist_thresh, float normal_max_degrees, float eps_angle, int max_iters)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    filterPointCloudByNormalAngle(cloud, pcl::Normal(0, 0, 1), normal_max_degrees, filtered_cloud);

    // Set up RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setMaxIterations(max_iters);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setEpsAngle(eps_angle);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_thresh);

    // Perform RANSAC segmentation
    seg.setInputCloud(filtered_cloud);
    seg.segment(*inliers, *coefficients);

    // Filter the pointcloud using the obtained inlier indices
    pcl::copyPointCloud(*(filterPointCloudByDistance(cloud, coefficients->values, dist_thresh)), *cloud);
}

}