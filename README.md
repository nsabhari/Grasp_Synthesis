# Grasp_Synthesis

This project was done as a part of Robot Manipulation Course at WPI.

It based on using the eye-in-hand system as an aid for grasp synthesis. Instead of using a partial point cloud data obtained from a single viewpoint, we use data from multiple viewpoints to get a more complete data on the target object. With more data available, it reduces the assumptions to be made on the complete geometry of the object. This gives us a higher probability of synthesizing a stable grasp for the object. Using 3 viewpoints gave ~20% of possible stable grasps. Having 9 viewpoints increased it to ~80%.

This project is based on the research paper “Viewpoint Optimization for Aiding Grasp Synthesis Algorithms using Reinforcement Learning” authored by B. Calli, W. Caarls, M. Wisse and P. Jonker

pcl_test.zip : Contains the files used for the project. <br />

Following C++ file are used:<br />
1) 1_Transform.cpp : Used to tranform the point cloud from Kinect to World frame<br />
2) 2_Fuse.cpp : Used to fuse the point clouds from multiple viewpoints<br />
3) 3_Viewpoint.cpp : Used to change the Kinect position in gazebo<br />
4) 4_Filter_Segment.cpp : Filtering and segmenting out the object<br />
5) 5_Normals.cpp : Finding normals and finding the final grasp config.<br />

models.zip : These contain the object models used in the simulation. These need to be placed in Home/.gazebo/models folder.<br />