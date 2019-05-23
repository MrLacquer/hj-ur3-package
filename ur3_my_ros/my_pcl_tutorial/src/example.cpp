#include <ros/ros.h>
#include <stdio.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>


ros::Publisher pub;

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
      pcl::PCLPointCloud2 cloud_filtered;

      // Perform the actual filtering
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (0.1f, 0.1f, 0.01f);
      sor.filter (cloud_filtered);

      // Publish the data
      pub.publish (cloud_filtered);
}

int main (int argc, char** argv)
{
      printf("====== pcl filtering Start! ======\n");

      // Initialize ROS
      ros::init (argc, argv, "my_pcl_tutorial");
      ros::NodeHandle nh;

      // Create a ROS subscriber for the input point cloud
      ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
      //ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

      // Create a ROS publisher for the output point cloud
      pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

      // Spin
      ros::spin ();
}