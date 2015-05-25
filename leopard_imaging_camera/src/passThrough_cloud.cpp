#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

class PassThrough
{


    //image_transport::Publisher image_pub;
    ros::Publisher pub;
    ros::Subscriber sub;
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);


    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::visualization::PCLVisualizer* pviewer;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pcloud;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pcloud_filtered;
    std::vector<int> indecies;

    public:

    PassThrough(ros::NodeHandle n) :
        pcloud(new pcl::PointCloud<pcl::PointXYZ>),
        pcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
    {

        sub = n.subscribe("/stereo/points2", 1, &PassThrough::cloud_cb, this);
        pub = n.advertise<sensor_msgs::PointCloud2> ("/stereo/points2_passed", 1);
        //pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/stereo/points2_filtered", 1);

        pass.setFilterFieldName ("z");
        pass.setFilterLimits (1.0, 4.0);

        pviewer = new pcl::visualization::PCLVisualizer("myViewer");
        pviewer->setBackgroundColor(0,0,0);
        pviewer->addCoordinateSystem(1.0);
        pviewer->initCameraParameters();

    }

};

void PassThrough::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  double begin = ros::Time::now().toSec();
  // Container for original & filtered data
  
  //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //pcl::PCLPointCloud2 cloud_filtered;

  //pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  
  // Convert to PCL data type
//  pcl_conversions::toPCL(*input, cloud); //when using PCLPoinyCloud2
  //pcl::fromROSMsg(*input, cloud); //when using PointXYZ
  double end_beforecvtpcl = ros::Time::now().toSec();
  ROS_INFO("tictic_beforecvtpcl: %lf", end_beforecvtpcl-begin);
  pcl::fromROSMsg(*input, *pcloud); //when using PointXYZ
  double end_cvtpcl = ros::Time::now().toSec();
  ROS_INFO("tictic_cvtpcl: %lf", end_cvtpcl-begin);


  // Perform the actual filtering
  /*pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);
    */

  //remove NaN from input cloud
  //pcl::removeNaNFromPointCloud(cloud, cloud, indecies);
  pcl::removeNaNFromPointCloud(*pcloud, *pcloud, indecies);

  //pass.setInputCloud(cloud.makeShared());
  pass.setInputCloud(pcloud);
  //pass.filter(cloud_filtered);
  pass.filter(*pcloud_filtered);


  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  //TODO how to convert from PointXYZ to ros?
  //I think you must have a PCLPointCloud2 object
  //pcl_conversions::fromPCL(cloud_filtered, output);
  pcl::toROSMsg(*pcloud_filtered, output);

  // Publish the data
  pub.publish(output);
  //pviewer->showCloud(cloud_filtered.makeShared());


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pcloud_filtered, 0, 255, 0);

  //update point cloud. if it does not exists already, then create it
  if (!pviewer->updatePointCloud<pcl::PointXYZ> (pcloud_filtered, single_color, "cloud1")){

      pviewer->addPointCloud<pcl::PointXYZ> (pcloud_filtered, single_color, "cloud1");
      //pviewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
  }

  pviewer->spinOnce();


}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "passThrough_cloud");
  ros::NodeHandle nh;

  PassThrough passThrough_cloud(nh);


  // Spin
  ros::spin ();
}
