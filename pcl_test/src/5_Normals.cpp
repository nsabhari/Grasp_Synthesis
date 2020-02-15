#include <ros/ros.h>
#include <math.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// For Normal Estimation
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <fstream>

class Normal_Estimation
{
private:
  ros::Subscriber sub_Obj;
  //ros::Publisher pub_Nor_All;
  //ros::Publisher pub_Nor_FP;
  //ros::Publisher pub_Nor_SP;
  //ros::Publisher pub_Nor_TP;
  float Threshold;
  float Dis_Thresh;
  float Threshold_a;
  int nPcl;
  int Visual;
  Eigen::Vector4f xyz_centroid;
  pcl::PointCloud<pcl::PointXYZRGBNormal> pclT_cld_nor;
  pcl::PointCloud<pcl::PointXYZRGBNormal> pclT_pair_1;
  pcl::PointCloud<pcl::PointXYZRGBNormal> pclT_pair_2;
  pcl::PointCloud<pcl::PointXYZ> pclT_LinePts;

public:
  Normal_Estimation(ros::NodeHandle *nh)
  {
    sub_Obj = nh->subscribe ("/processed/seg", 1, &Normal_Estimation::normal_cb, this);
    //pub_Nor_All = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/normal_all", 1);
    //pub_Nor_FP = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/normal_FP", 1);
    //pub_Nor_SP = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/normal_SP", 1);
    //pub_Nor_TP = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/processed/normal_TP", 1);
    Threshold = 10.0;
    Threshold_a = 10.0;
    Dis_Thresh = 0.075;
    Visual = 1;
  } 
  
  void normal_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud_msg)
  {    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclT_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);           // PCLT : Received data
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudPtr(pclT_cloud);                                   // PCLT : Received data for input to ne
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclT_pair_1A(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclT_pair_1B(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclT_pair_2A(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclT_pair_2B(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pclT_pair_3(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pclT_LinePts(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudPtr_1(pclT_LinePts);

    *pclT_cloud = *cloud_msg;
    // Create and accumulate points
    
    compute3DCentroid (*pclT_cloud, xyz_centroid);
    pclT_LinePts.width = 6;
    pclT_LinePts.height = 1;
    pclT_LinePts.points.resize (pclT_LinePts.width * pclT_LinePts.height);
    for (int k = 0; k < pclT_LinePts.width; k++)
    {
      pclT_LinePts.points[k].x = xyz_centroid[0]-0.2;
      pclT_LinePts.points[k].y = xyz_centroid[1];
      pclT_LinePts.points[k].z = xyz_centroid[2]; 
    }
    
    pclT_LinePts.points[1].x = pclT_LinePts.points[0].x+0.1;
    pclT_LinePts.points[2].y = pclT_LinePts.points[0].y+0.1;
    pclT_LinePts.points[3].z = pclT_LinePts.points[0].z+0.1;
    pclT_LinePts.points[4].x = pclT_LinePts.points[0].x+0.2;
    pclT_LinePts.points[5].x = pclT_LinePts.points[0].x-0.05;
    pclT_LinePts.points[5].y = pclT_LinePts.points[0].y-0.2;
    pclT_LinePts.points[5].z = pclT_LinePts.points[0].z+0.2;
    // std::cout << xyz_centroid << std::endl;

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloudPtr);

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setSearchMethod (tree);

    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    //ne.setRadiusSearch (0.01);
    ne.setViewPoint (xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);
    ne.setKSearch (10);
    ne.compute (*normals);
    pcl::concatenateFields(*pclT_cloud, *normals, pclT_cld_nor);

    // ofstream myfile;
    // myfile.open ("Test123.txt");
    // for(int i = 0; i< pclT_cld_nor.points.size(); i++)
    // {
    //   myfile << "Point "<< i << "\t" << pclT_cld_nor.points[i].x << "\t" << pclT_cld_nor.points[i].y << "\t" << pclT_cld_nor.points[i].z << "\t" <<
    //     pclT_cld_nor.points[i].normal_x << "\t" << pclT_cld_nor.points[i].normal_y << "\t" << pclT_cld_nor.points[i].normal_z << "\n";
    // }
    // myfile.close();

    // pub_Nor_All.publish (*pclT_cloud);

    // FIRST ITERATION
    nPcl = pclT_cld_nor.points.size();
    float v3_1 = 0;
    float v3_2 = 0;
    float v3_3 = 0;
    float v3_mag = 0;
    float v1_dot_v3 = 0;
    float v2_dot_v3 = 0;
    float A1=0;
    float A2=0;
    for (int i = 0; i<nPcl-1; i++)
    {
      for (int j=i+1; j<nPcl; j++)
      {
        v3_1 = pclT_cld_nor.points[i].x - pclT_cld_nor.points[j].x;
        v3_2 = pclT_cld_nor.points[i].y - pclT_cld_nor.points[j].y;
        v3_3 = pclT_cld_nor.points[i].z - pclT_cld_nor.points[j].z;
        v3_mag = sqrt((v3_1*v3_1)+(v3_2*v3_2)+(v3_3*v3_3));
        v1_dot_v3 = v3_1*pclT_cld_nor.points[i].normal_x + v3_2*pclT_cld_nor.points[i].normal_y + v3_3*pclT_cld_nor.points[i].normal_z;
        v2_dot_v3 = v3_1*pclT_cld_nor.points[j].normal_x + v3_2*pclT_cld_nor.points[j].normal_y + v3_3*pclT_cld_nor.points[j].normal_z;
        A1 = fabs(acos(v1_dot_v3/v3_mag)*180/M_PI);
        A2 = fabs(acos(v2_dot_v3/v3_mag)*180/M_PI);

        if (((A1<=Threshold)&&(A2>=(180-Threshold))&&(v3_mag<=Dis_Thresh))||(A2<=Threshold)&&(A1>=(180-Threshold))&&(v3_mag<=Dis_Thresh))
        { 
          pclT_pair_1A->push_back(pclT_cld_nor.points[i]);
          pclT_pair_1B->push_back(pclT_cld_nor.points[j]);
          break;
        } 

      }
    }
    
    // pclT_pair_1 = *pclT_pair_1A + *pclT_pair_1B; 
    // pub_Nor_FP.publish (*pclT_pair_1A);

    // SECOND ITERATION
    nPcl = pclT_pair_1A->points.size();
    float app_dot_v3 = 0;

    for (int i = 0; i<nPcl-1; i++)
    {
      v3_1 = pclT_pair_1A->points[i].x - pclT_pair_1B->points[i].x;
      v3_2 = pclT_pair_1A->points[i].y - pclT_pair_1B->points[i].y;
      v3_3 = pclT_pair_1A->points[i].z - pclT_pair_1B->points[i].z;
      v3_mag = sqrt((v3_1*v3_1)+(v3_2*v3_2)+(v3_3*v3_3));
      app_dot_v3 = v3_1*1 + v3_2*0 + v3_3*0;
      A1 = fabs(acos(app_dot_v3/v3_mag)*180/M_PI);

      if ((A1<=(90+Threshold_a))&&(A1>=(90-Threshold_a)))
      { 
        pclT_pair_2A->push_back(pclT_pair_1A->points[i]);
        pclT_pair_2B->push_back(pclT_pair_1B->points[i]);
      } 
    }
    
    // pclT_pair_2 = *pclT_pair_2A + *pclT_pair_2B;
    // pub_Nor_SP.publish (pclT_pair_2);

    // THIRD ITERATION
    nPcl = pclT_pair_2A->points.size();
    float min_dis = 999;
    float dis;
    int index = 0;
    float v4_1 = 0;
    float v4_2 = 0;
    float v4_3 = 0;
    // float v4_mag = 0;
    float v3_x_v4_1 = 0;
    float v3_x_v4_2 = 0;
    float v3_x_v4_3 = 0;
    float v3_x_v4_mag = 0;
    for (int i = 0; i<nPcl-1; i++)
    { 
      v3_1 = pclT_pair_2A->points[i].x - pclT_pair_2B->points[i].x;
      v3_2 = pclT_pair_2A->points[i].y - pclT_pair_2B->points[i].y;
      v3_3 = pclT_pair_2A->points[i].z - pclT_pair_2B->points[i].z;
      v3_mag = sqrt((v3_1*v3_1)+(v3_2*v3_2)+(v3_3*v3_3));

      v4_1 = pclT_pair_2A->points[i].x - xyz_centroid[0];
      v4_2 = pclT_pair_2A->points[i].y - xyz_centroid[1];
      v4_3 = pclT_pair_2A->points[i].z - xyz_centroid[2];
      // v4_mag = sqrt((v4_1*v4_1)+(v4_2*v4_2)+(v4_3*v4_3));

      v3_x_v4_1 = v3_2*v4_3 - v3_3*v4_2;
      v3_x_v4_2 = v3_3*v4_1 - v3_1*v4_3;
      v3_x_v4_3 = v3_1*v4_2 - v3_2*v4_1;
      v3_x_v4_mag = sqrt((v3_x_v4_1*v3_x_v4_1)+(v3_x_v4_2*v3_x_v4_2)+(v3_x_v4_3*v3_x_v4_3));

      dis = fabs(v3_x_v4_mag/v3_mag);

      //dis = fabs(pclT_pair_2A->points[i].z - xyz_centroid[2]);
      if (dis < min_dis)
      {
        min_dis = dis;
        index = i;
      }
    }

    if (nPcl > 0)
    {
      pclT_pair_3->push_back(pclT_pair_2A->points[index]);
      pclT_pair_3->push_back(pclT_pair_2B->points[index]);
    }

    // pub_Nor_SP.publish (*pclT_pair_3);

    std::cout << "No. normal pairs within threshold : " << pclT_pair_1A->points.size() << std::endl;
    std::cout << "No. of pairs perpendicular to approach vector : "<< pclT_pair_2A->points.size() << std::endl;
    // std::cout << pclT_pair_3->points.size() << std::endl;

    if (Visual==1)
    { 
      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.initCameraParameters ();
      // viewer.addCoordinateSystem (1.0);
      
      int vp1(0);
      viewer.createViewPort(0.0,0.5,0.5,1.0,vp1);
      viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pclT_cloud,normals,1,0.01,"Cloud_N1",vp1);

      int vp2(0);
      viewer.createViewPort(0.5,0.5,1.0,1.0,vp2);
      if (pclT_pair_1A->points.size() > 0)
      {
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(pclT_pair_1A,1,0.01,"Cloud_N1A",vp2);
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(pclT_pair_1B,1,0.01,"Cloud_N1B",vp2);
      }

      int vp3(0);
      viewer.createViewPort(0.0,0.0,0.5,0.5,vp3);
      if (pclT_pair_2A->points.size() > 0)
      {
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(pclT_pair_2A,1,0.01,"Cloud_N2A",vp3);
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(pclT_pair_2B,1,0.01,"Cloud_N2B",vp3);
      }

      int vp4(0);
      viewer.createViewPort(0.5,0.0,1,0.5,vp4);
      if (pclT_pair_3->points.size() > 0)
      {
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(pclT_pair_3,1,0.015,"Cloud_N3",vp4);
        viewer.addSphere<pcl::PointXYZRGBNormal>(pclT_pair_3->points[0],0.005,0,1,0,"Contact_1",vp4);
        viewer.addSphere<pcl::PointXYZRGBNormal>(pclT_pair_3->points[1],0.005,0,1,0,"Contact_2",vp4);
      }

      viewer.addPointCloud<pcl::PointXYZRGB>(pclT_cloud, "Original Cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0,"Original Cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Original Cloud");
      viewer.addLine<pcl::PointXYZ>(pclT_LinePts.points[0],pclT_LinePts.points[1],1,0,0,"LineX");
      viewer.addLine<pcl::PointXYZ>(pclT_LinePts.points[0],pclT_LinePts.points[2],0,1,0,"LineY");
      viewer.addLine<pcl::PointXYZ>(pclT_LinePts.points[0],pclT_LinePts.points[3],0,0,1,"LineZ");
      viewer.addSphere<pcl::PointXYZ>(pclT_LinePts.points[4],0.0050,0,0,1,"COG");
      viewer.addSphere<pcl::PointXYZ>(pclT_LinePts.points[0],0.0025,1,1,1,"Axis");
      viewer.setBackgroundColor (0.5, 0.5, 0.5);

      viewer.spinOnce ();
      viewer.setCameraPosition(pclT_LinePts.points[5].x,pclT_LinePts.points[5].y,pclT_LinePts.points[5].z,
                               pclT_LinePts.points[4].x,pclT_LinePts.points[4].y,pclT_LinePts.points[4].z,
                               0.33,0.33,0.33);
      // viewer.resetCamera  ();

      // std::vector<pcl::visualization::Camera> cam; 
      // viewer.getCameras(cam);
      // std::cout << cam[0].pos[0] << "\t" << cam[0].pos[1] << "\t" << cam[0].pos[3] << std::endl;
      // std::cout << cam[0].view[0] << "\t" << cam[0].view[1] << "\t" << cam[0].view[3] << std::endl;
      // std::cout << cam[0].focal[0] << "\t" << cam[0].focal[1] << "\t" << cam[0].focal[3] << std::endl;

      while (!viewer.wasStopped ())  // THE ORGINAL !viewer.wasStopped () 
      {
        viewer.spinOnce ();

      }
    }

    // std::cout << "Press enter to continue" << std::endl;
    // std::string inputString;
    // std::getline(std::cin, inputString);
  }
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PCL_Normals");
  ros::NodeHandle nh;

  Normal_Estimation NEObject = Normal_Estimation(&nh);

  // Spin
  ros::spin ();
}