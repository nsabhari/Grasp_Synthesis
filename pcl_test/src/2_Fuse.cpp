#include <ros/ros.h>
#include <std_msgs/Int32.h>
// PCL specific includes
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class Fuse_Viewpoints
{
private:
  ros::Publisher pub_Fused;
  ros::Subscriber sub_Trf;
  ros::Subscriber sub_Flag;
  int Flag;
  pcl::PointCloud<pcl::PointXYZRGB> pclT_cloud_last;
public:
  Fuse_Viewpoints(ros::NodeHandle *nh)
  {
    Flag = 0;
    sub_Trf = nh->subscribe ("/processed/transformed", 1, &Fuse_Viewpoints::fuse_cb, this);
    sub_Flag = nh->subscribe ("/processed/Kinect_Flag", 1, &Fuse_Viewpoints::flag_cb, this);
    pub_Fused = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/fused", 1);
  } 
  void flag_cb (const std_msgs::Int32& Int_msg)
  {
    Flag = Int_msg.data;
  }
  void fuse_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_msg)
  { 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclT_cloud_fused(new pcl::PointCloud<pcl::PointXYZRGB>);      // PCLT : Fused data
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudPtr(pclT_cloud_fused);                              // PCLT : Fused data for input to Voxel
    pcl::PointCloud<pcl::PointXYZRGB> pclT_cloud_vds;                                                    // PCLT : Voxel DownSampled data

    if (Flag == 0)
    {
      *pclT_cloud_fused = *cloud_msg;
      
      if (pclT_cloud_last.width>0)
      {
        *pclT_cloud_fused += pclT_cloud_last;                                     // Fusing point clouds 
      }

      // Voxel DownSampling after fusing it
      pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.005, 0.005, 0.005);
      sor.filter (pclT_cloud_vds);

      pclT_cloud_last = pclT_cloud_vds;                                           // Storing the last point cloud

      //std::cout << pclT_cloud_vds << std::endl;

      // Publish the data
      pub_Fused.publish (pclT_cloud_last);
    }
    else
    { //std::cout << "ELSE" << std::endl;
      if (pclT_cloud_last.width>0)
      {
        // Publish the data
        pub_Fused.publish (pclT_cloud_last);
      }
    }
  }
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PCL_Fuse");
  ros::NodeHandle nh;

  Fuse_Viewpoints FWObject = Fuse_Viewpoints(&nh);

  // Spin
  ros::spin ();
}