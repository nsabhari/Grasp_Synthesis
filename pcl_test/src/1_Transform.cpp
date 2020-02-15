#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/GetModelState.h>
// PCL specific includes
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
ros::ServiceClient get_Kinect_pos;

gazebo_msgs::GetModelState model_state;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclT_cloud_trf(new pcl::PointCloud<pcl::PointXYZRGB>);    // PCLT : Transfromed data
pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudPtr(pclT_cloud_trf);                            // PCLT : Transformed data for input to Voxel
pcl::PointCloud<pcl::PointXYZRGB> pclT_cloud_vds;                                                // PCLT : Voxel DownSampled dat

void 
cloud_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_msg)
{
  // Transforming the filtered pointcloud
  model_state.request.model_name = "Kinect";
  get_Kinect_pos.call(model_state);								// Getting the Kinect pose from Gazebo

  double tx,ty,tz, qx, qy, qz, qw, roll, pitch, yaw;
  tx = model_state.response.pose.position.x;
  ty = model_state.response.pose.position.y;
  tz = model_state.response.pose.position.z;
  qx = model_state.response.pose.orientation.x;
  qy = model_state.response.pose.orientation.y;
  qz = model_state.response.pose.orientation.z;
  qw = model_state.response.pose.orientation.w;

  tf::Quaternion q(qx, qy, qz, qw);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);									// Getting R,P,Y from quaternion data
  //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

  // CAlculating the transformation matirx
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << tx, ty, tz;
  transform.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitY()));
  transform.rotate (Eigen::AngleAxisf (-M_PI/2+roll, Eigen::Vector3f::UnitZ()));
  transform.rotate (Eigen::AngleAxisf (-yaw, Eigen::Vector3f::UnitY()));
  transform.rotate (Eigen::AngleAxisf (-pitch, Eigen::Vector3f::UnitX()));
  
  //std::cout << transform.matrix() << std::endl;

  pcl::transformPointCloud (*cloud_msg, *pclT_cloud_trf, transform);	// Executing the transformation

  // Voxel DownSampling after transforming it
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.005, 0.005, 0.005);
  sor.filter (pclT_cloud_vds);

  // Publish the data
  pub.publish (pclT_cloud_vds);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PCL_Transform");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
  get_Kinect_pos = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/transformed", 1);

  // Spin
  ros::spin ();
}