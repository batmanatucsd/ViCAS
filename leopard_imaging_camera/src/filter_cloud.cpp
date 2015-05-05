#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/statistical_outlier_removal.h>



class FilterCloud
{


    //image_transport::Publisher image_pub;
    ros::Publisher pub;
    ros::Subscriber sub;
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    public:

    FilterCloud(ros::NodeHandle n)
    {

        sub = n.subscribe("/stereo/points2", 1, &FilterCloud::cloud_cb, this);
        pub = n.advertise<sensor_msgs::PointCloud2> ("/stereo/points2_filtered", 1);
        //pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/stereo/points2_filtered", 1);

        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);

    }

};

void FilterCloud::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    ROS_INFO("got a cloud!");
  // Container for original & filtered data
  
  //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //pcl::PCLPointCloud2 cloud_filtered;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  
  // Convert to PCL data type
//  pcl_conversions::toPCL(*input, cloud); //when using PCLPoinyCloud2
  pcl::fromROSMsg(*input, cloud); //when using PointXYZ

  // Perform the actual filtering
  /*pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);
    */

  //remove NaN from input cloud
  std::vector<int> indecies;
  pcl::removeNaNFromPointCloud(cloud, cloud, indecies);

  sor.setInputCloud(cloud.makeShared());
  sor.setMeanK (3);
  sor.setStddevMulThresh (1.0);
  sor.filter(cloud_filtered);


  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  //TODO how to convert from PointXYZ to ros?
  //I think you must have a PCLPointCloud2 object
  //pcl_conversions::fromPCL(cloud_filtered, output);
  pcl::toROSMsg(cloud_filtered, output);

  // Publish the data
  pub.publish(output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "filter_cloud");
  ros::NodeHandle nh;

  FilterCloud filter_cloud(nh);


  // Spin
  ros::spin ();
}
