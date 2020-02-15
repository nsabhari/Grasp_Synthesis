#include <ros/ros.h>
#include <std_msgs/Int32.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
// For Segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class Filter_Segment
{
private:
  ros::Publisher pub_Filter;
  ros::Publisher pub_Plane;
  ros::Publisher pub_Seg;
  ros::Subscriber sub_Fused;
  pcl::ModelCoefficients coefficients;
  pcl::PointCloud<pcl::PointXYZRGB> pclT_plane;
  pcl::PointCloud<pcl::PointXYZRGB> pclT_obj;
public:
  Filter_Segment(ros::NodeHandle *nh)
  {
    sub_Fused = nh->subscribe ("/processed/fused", 1, &Filter_Segment::filter_cb, this);
    pub_Filter = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/filter", 1);
    pub_Plane = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/plane", 1);
    pub_Seg = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/seg", 1);
  } 
  
  void filter_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_msg)
  { 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclT_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);      // PCLT : Received data
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudPtr(pclT_cloud);                              // PCLT : Received data for input to PassThrough
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    *pclT_cloud = *cloud_msg;
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (2.7, 3.3);
    pass.filter (*pclT_cloud);

    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.5, 1);
    pass.filter (*pclT_cloud);

    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.25, 0.25);
    // pass.setFilterLimits (-0.65, -0.35);  // Cube //0.65
    pass.filter (*pclT_cloud);

    pub_Filter.publish (*pclT_cloud);

    // Plane Segmentation
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloudPtr);
    seg.segment (*inliers, coefficients);
    
    copyPointCloud (*pclT_cloud, *inliers, pclT_plane);
    pub_Plane.publish (pclT_plane);

    // Remove table and get objects
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloudPtr);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (pclT_obj);

    //std::cout << pclT_obj << std::endl;

    pub_Seg.publish (pclT_obj);
  }
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PCL_Filter");
  ros::NodeHandle nh;

  Filter_Segment FSObject = Filter_Segment(&nh);

  // Spin
  ros::spin ();
}