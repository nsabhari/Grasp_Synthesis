#include <ros/ros.h>
#include <math.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>

ros::Publisher pub_Kinect;
ros::Publisher pub_Flag;
float step_size = 22.5;
float start = 0;
float end = 0;
unsigned int microseconds = 1000000;

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PCL_Viewpoint");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  // Create a ROS publisher for the Kinect Pose
  pub_Kinect = nh.advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 1);
  pub_Flag = nh.advertise<std_msgs::Int32> ("/processed/Kinect_Flag", 1);

  // double Viewpoint[9][6] = {{3-1.5*cos(theta1),1.5*sin(theta1),1,0,0.523,-theta1},
  //                           {3-1.5*cos(theta2),1.5*sin(theta2),1,0,0.523,-theta2},
  //                           {3-1.5*cos(theta3),1.5*sin(theta3),1,0,0.523,-theta3},
  //                           {3-1.5*cos(theta4),1.5*sin(theta4),1,0,0.523,-theta4},
  //                           {3-1.5*cos(theta5),1.5*sin(theta5),1,0,0.523,-theta5},
  //                           {3-1.5*cos(theta6),1.5*sin(theta6),1,0,0.523,-theta6},
  //                           {3-1.5*cos(theta7),1.5*sin(theta7),1,0,0.523,-theta7},
  //                           {3-1.5*cos(theta8),1.5*sin(theta8),1,0,0.523,-theta8},
  //                           {3-1.5*cos(theta9),1.5*sin(theta9),1,0,0.523,-theta9}};

  for (float i = start; i<=end; i=i+step_size)
  {
    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 m_rot;
    m_rot.setEulerYPR(-i*M_PI/180, 0.523, 0);

    // Convert into quaternion
    tf::Quaternion quat;
    m_rot.getRotation(quat);

    gazebo_msgs::ModelState Model_State;
    Model_State.model_name = "Kinect";
    Model_State.reference_frame = "world";
    Model_State.pose.position.x = 3-1.5*cos(i*M_PI/180);
    Model_State.pose.position.y = 1.5*sin(i*M_PI/180);
    Model_State.pose.position.z = 1;
    Model_State.pose.orientation.x = quat.x();
    Model_State.pose.orientation.y = quat.y();
    Model_State.pose.orientation.z = quat.z();
    Model_State.pose.orientation.w = quat.w();

    std_msgs::Int32 Flag;

    Flag.data = 1;
    pub_Flag.publish(Flag);
    ros::spinOnce();
    usleep(microseconds);
    pub_Kinect.publish(Model_State);
    ros::spinOnce();
    std::cout << "Viewpoint " << i << " Set" << std::endl;
    usleep(microseconds);
    Flag.data = 0;
    pub_Flag.publish(Flag);
    ros::spinOnce();
    usleep(microseconds);

    //std::cout << "Press enter to continue" << std::endl;
    //std::string inputString;
    //std::getline(std::cin, inputString);
  }
}