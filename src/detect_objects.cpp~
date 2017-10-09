/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
# include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <sstream> 
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <std_msgs/String.h>

//#include <iostream> // TODO: remove this dep
//rosrun pcl_ros pointcloud_to_pcd input:=/cloud_filtered_red _prefix:=/home/sampath/Documents/log/
//tf transform : rosrun tf static_transform_publisher 0 0 0 .1 0 0 0 /torso /camera_rgb_optical_frame 50

std_msgs::String output_string;

namespace point_cloud_lab
{



class PointCloudLab
{
private:

  ros::NodeHandle nh_;
  std::string action_name_;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher semifiltered_pub_;
  ros::Publisher object_pub_;
  ros::Publisher filtered_pub_red_;
  ros::Publisher filtered_pub_blue_;
  ros::Publisher filtered_pub_orange_;
  ros::Publisher filtered_pub_yellow_;
  ros::Publisher filtered_pub_green_;
  ros::Publisher filtered_pub_; // filtered point cloud for testing the algorithms
  ros::Publisher plane_pub_; // points that were recognized as part of the table
  ros::Publisher block_pose_pub_; // publishes to the block logic server
  tf::TransformListener tf_listener_;

  //std::vector<geometry_msgs::Pose> block_poses_;
  geometry_msgs::PoseArray block_poses_;

  // Parameters of problem
  std::string arm_link;
  double block_size;
  double table_height;

public:

  PointCloudLab(const std::string name) :
    action_name_(name)
  {
    ROS_INFO_STREAM("Starting Point Cloud Lab node 3");

    // Parameters
    arm_link = "torso";
    block_size = 0.4;
    table_height = 0.0;
    block_poses_.header.stamp = ros::Time::now();
    block_poses_.header.frame_id = arm_link;



    // publish positions
    object_pub_ = nh_.advertise<std_msgs::String>("obj_pos", 10);


    // Subscribe to point cloud
    point_cloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloudLab::cloudCallback, this);


    // publish the remaining points
    semifiltered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("semi_filter_output", 1);

    // publish the red points
    filtered_pub_red_= nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("red", 1);

    // publish the blue points
    filtered_pub_blue_= nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("blue", 1);

    // publish the orange points
    filtered_pub_orange_= nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("orange", 1);

    // publish the yellow points
    filtered_pub_yellow_= nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("yellow", 1);

    // publish the green points
    filtered_pub_green_= nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("green", 1);

    // Publish a point cloud of filtered data that was not part of table
    filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Publish a point cloud of data that was considered part of the plane
    plane_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane_output", 1);





  }

  pcl::PointCloud<pcl::PointXYZRGB> ros_msg_to_pcl(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
	/*
	input : takes the message from the ros topic "/camera/depth_registered/points"
	output : converts into point cloud and returns the cloud
	*/
	// initializing output variable cloud
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	// converts the message to point cloud of forma (x y z RGB)
        pcl::fromROSMsg(*msg, cloud);

	return cloud;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_to_baxters_axis(pcl::PointCloud<pcl::PointXYZRGB> cloud)
  {
   	/*
	input : the private class variable arm_link and the point cloud obtained from conversion
	output : transformed point cloud wrt baxter frame
	*/
	
	//output cloud initialization
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
	// time w.r.t ros
	ros::Time time_st = ros::Time(0);
	// listens to the transform for the duration of 1.0 unit in ros time
    	tf_listener_.waitForTransform(std::string(arm_link), cloud.header.frame_id,
                                  time_st, ros::Duration(1.0));
	//  if it could not convert into baxters frame return error
    	if(!pcl_ros::transformPointCloud(std::string(arm_link), cloud, *cloud_transformed, tf_listener_))
    	{
      	ROS_ERROR("Error converting to desired frame");
	return cloud_transformed;
    	}
	
	// return the point cloud
	return cloud_transformed;
	
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr limit_points_to_table(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed)
  {

   	/*
	input : the transformed point cloud wrt baxter's frame
	output : filtered point cloud that limit's to just table and blocks
	*/
	
	// filtering is done based on the z axis, so initializing the output cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredZ(new pcl::PointCloud<pcl::PointXYZRGB>);
	// instance of the passthrough filter from pcl
    	pcl::PassThrough<pcl::PointXYZRGB> pass;
	// set's the input cloud as the transformed cloud wrt baxter's frame
    	pass.setInputCloud(cloud_transformed);
	// filter it wrt z axis
    	pass.setFilterFieldName("z");
	// adjust the filter limits to table height and table_height _ block size with some threshold
    	pass.setFilterLimits(-0.5, 0.5);
	// finally execute the filtering
    	pass.filter(*cloud_filteredZ);

	// Limit to things in front of the robot (x axis filtering)
	
   	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredX(new pcl::PointCloud<pcl::PointXYZRGB>);
    	pass.setInputCloud(cloud_filteredZ);
    	pass.setFilterFieldName("x");
    	//pass.setFilterLimits(-0.7,0.7);
	pass.setFilterLimits(0.52,1);
    	pass.filter(*cloud_filteredX);

	// Limit to things in front of the robot (x axis filtering)
	
   	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    	pass.setInputCloud(cloud_filteredX);
    	pass.setFilterFieldName("y");
    	//pass.setFilterLimits(-0.3,0.1);
	pass.setFilterLimits(-5, 5);
    	pass.filter(*cloud_filtered);
	
	

	// return the filtered cloud
	return cloud_filtered;
	
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_grade_samples(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {

   	/*
	input : the point cloud
	output : downgrade the points and return the point cloud
	*/
	
	// filtering is done based on the z axis, so initializing the output cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	// instance of the voxelgrid filter from pcl
    	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	// set's the input cloud
    	sor.setInputCloud (cloud);
	// filter it to the thresholded distance points of x,y,z axis
    	sor.setLeafSize (0.008f, 0.008f, 0.008f);
	// finally execute the filtering
    	sor.filter (*cloud_filtered);

	// return the filtered cloud
	return cloud_filtered;
	
  }

  void check_if_any_point_exists(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {

   	/*
	input : the point cloud
	output : nothing, just print the points size
	*/

	// Check if any points remain
    	if( cloud->points.size() == 0 )
    	{
      	ROS_ERROR("0 points left");
      	return;
    	}
    	else
    	{
      	//ROS_INFO("[block detection] Filtered, %d points left", (int) cloud->points.size());
    	}
	
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_segmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
   	/*
	input : the point cloud (filtered)
	output : return point cloud by removing the planar points
		 plane equation is of the form ax + by +cz +d = 0
		 prints the coeficients of a,b,c and d
	*/

        // Segment components -----------

       // Create the segmentation object for the planar model and set all the parameters
       pcl::SACSegmentation<pcl::PointXYZRGB> seg;
       seg.setOptimizeCoefficients(true);
       seg.setModelType(pcl::SACMODEL_PLANE);
       seg.setMethodType(pcl::SAC_RANSAC); // robustness estimator - RANSAC is simple
       seg.setMaxIterations(200);
       seg.setDistanceThreshold(0.005); // determines how close a point must be to the model in order to be considered an inlier

       pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
       pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    
       int nr_points = cloud->points.size();

       // Segment cloud until there are less than 30% of points left? not sure why this is necessary
       while(cloud->points.size() > 0.3 * nr_points)
       {

         // Segment the largest planar component from the remaining cloud (find the table)
         seg.setInputCloud(cloud);
         seg.segment(*inliers, *coefficients);

         if(inliers->indices.size() == 0)
         {
          ROS_ERROR("[Plane detection] Could not estimate a planar model for the given dataset.");
          
       }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Write the planar inliers to disk
      extract.filter(*cloud_plane);

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud);

      // Show the contents of the inlier set, together with the estimated plane parameters, in ax+by+cz+d=0 form (general equation of a plane)
      //std::cerr << "Model coefficients: " << coefficients->values[0] << " "
      //         << coefficients->values[1] << " "
      //          << coefficients->values[2] << " "
      //          << coefficients->values[3] << std::endl;
    }

    // publish the plane detected into global variable plane_pub_
    plane_pub_.publish(cloud_plane);
    filtered_pub_.publish(cloud);
    
    return cloud;
	
  }

  std::vector <pcl::PointIndices> colored_clusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {

   	/*
	input : the point cloud
	output : different clusters of colors (cluster array) 
		 each index represents a cluster
	*/
	// initialize output variable cluster	
	std::vector <pcl::PointIndices> clusters;
	// initialization of tree
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	// initialization of method
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	// set's the input cloud
    	reg.setInputCloud (cloud);
	// search method tree
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (1);
	reg.setPointColorThreshold (6);
	reg.setRegionColorThreshold (5);
	reg.setMinClusterSize (0);
 	reg.extract (clusters);

	ROS_INFO_STREAM("[cluster indices] : " << clusters.size());

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	
	filtered_pub_red_.publish(colored_cloud);

	// return the filtered cloud
	return clusters;
	
  }


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_outliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {

   	/*
	input : the point cloud
	output : returns the point cloud by removing outliers
	*/

	if (cloud->points.size() == 0)
		return cloud;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	// instance of the RadiusOutlierRemoval filter from pcl
    	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	// set's the input cloud
    	outrem.setInputCloud (cloud);
	// filter it to the thresholded radius
    	outrem.setRadiusSearch(0.06);
	outrem.setMinNeighborsInRadius (15);

	// finally execute the filtering
    	outrem.filter (*cloud_filtered);

	// return the filtered cloud
	return cloud_filtered;
	
  }


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr check_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , int rMax , int gMax , int bMax, int rMin, int gMin, int bMin)
  {
  // Filter by red color
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_color (new pcl::PointCloud<pcl::PointXYZRGB>);
  // build the condition 
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ()); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LE, rMax))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GE, rMin))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LE, gMax))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GE, gMin))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LE, bMax))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GE, bMin))); 
   
   // build the filter 
   pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
   condrem.setCondition(color_cond); 
   condrem.setInputCloud (cloud); 
   // apply filter 
   condrem.filter (*cloud_filtered_color); 

   
   return cloud_filtered_color;

  }

  std::string check_block_dimensions(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
       /*
	input : the point cloud
	output : the coordinates of the block
	functionality: find the outer dimensions of the block
       */

      if (cloud->points.size() == 0)
		return "";

      // find the outer dimensions of the cluster
      float xmin = 0; float xmax = 0;
      float ymin = 0; float ymax = 0;
      float zmin = 0; float zmax = 0;
      for(size_t i = 0; i < cloud->points.size(); i++)
      {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;
        if(i == 0)
        {
          xmin = xmax = x;
          ymin = ymax = y;
          zmin = zmax = z;
        }
        else
        {
          xmin = std::min(xmin, x);
          xmax = std::max(xmax, x);
          ymin = std::min(ymin, y);
          ymax = std::max(ymax, y);
          zmin = std::min(zmin, z);
          zmax = std::max(zmax, z);
        }
      }

      // Check if these dimensions make sense for the block size specified
      float xside = xmax-xmin;
      float yside = ymax-ymin;
      float zside = zmax-zmin;

      //ROS_INFO_STREAM("[block detection] xside: " << xside << " yside: " << yside << " zside " << zside );

      const float tol = 0.01; // 1 cm error tolerance
      // In order to be part of the block, xside and yside must be between
      // blocksize and blocksize*sqrt(2)
      // z must be equal to or smaller than blocksize
      if(xside < block_size*sqrt(2)+tol &&
         yside < block_size*sqrt(2)+tol &&
                 zside > tol )
      {
        // If so, then figure out the position and the orientation of the block
        float angle = atan(block_size/((xside+yside)/2));

        if(yside < block_size)
          angle = 0.0;

	//ROS_INFO_STREAM("[block detection] xside: " << xside << " yside: " << yside << " zside " << zside << " angle" << angle );
	float x = xmin+(xside)/2.0 ;
	float y = ymin+(yside)/2.0 ;
	float z = zmax - block_size/2.0;
 	std:: string dimensions = addBlock( x, y, z, angle);   
       	return dimensions;
      }
	return "";
  }

  void extract_colors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {

   	/*
	input : the point cloud
	output : none,
	functionality: get the green,red,blue,yellow and orange colors
		       check whether they are blocks
	*/
	
	std::string str = "";
	std::string dimensions_red = "";
	std::string dimensions_blue = "";
	std::string dimensions_green = "";
	std::string dimensions_orange = "";
	std::string dimensions_yellow = "";
	int r_max ;
	int r_min ;
	int g_max;
	int g_min;
	int b_max;
	int b_min;

	// red cloud
	r_max = 130;
	g_max = 60;
	b_max = 70;
	r_min = 45;	
	g_min = 0;
	b_min = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_red = PointCloudLab::check_color(cloud ,r_max,g_max,b_max,r_min,g_min,b_min);
	if (cloud_red->points.size() != 0)
		{
		cloud_red = remove_outliers(cloud_red); //removes outliers
		dimensions_red = check_block_dimensions(cloud_red);
		filtered_pub_red_.publish(cloud_red);
		}

	
	
	// green cloud
	r_max = 50;
	g_max = 150;
	b_max = 50;//30;
	r_min = 0;
	g_min = 40;
	b_min = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_green = PointCloudLab::check_color(cloud ,r_max,g_max,b_max,r_min,g_min,b_min);
	if (cloud_green->points.size() != 0)
		{
		cloud_green = remove_outliers(cloud_green);
		dimensions_green = check_block_dimensions(cloud_green);
	        filtered_pub_green_.publish(cloud_green);
		}

	
	
	//blue cloud
	/*
	r_max = 35;
	g_max = 40;
	b_max = 100;
	r_min = 0;
	g_min = 0;
	b_min = 45;
	*/
	r_max = 50;
	g_max = 80;
	b_max = 255;
	r_min = 0;
	g_min = 0;
	b_min = 60;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blue = PointCloudLab::check_color(cloud ,r_max,g_max,b_max,r_min,g_min,b_min);
	if (cloud_blue->points.size() != 0)
		{
		cloud_blue = remove_outliers(cloud_blue);
		dimensions_blue = check_block_dimensions(cloud_blue);
		filtered_pub_blue_.publish(cloud_blue);
		}

	//orange cloud
	r_max = 255;
	g_max = 210;
	b_max = 130;
	r_min = 90;
	g_min = 50;
	b_min = 20;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orange = PointCloudLab::check_color(cloud ,r_max,g_max,b_max,r_min,g_min,b_min);
	if (cloud_orange->points.size() != 0)
		{
		cloud_orange = remove_outliers(cloud_orange);
		dimensions_orange = check_block_dimensions(cloud_orange);
		filtered_pub_orange_.publish(cloud_orange);
		}

	//yellow cloud
	r_max = 256;
	g_max = 256;
	b_max = 215;
	r_min = 250;
	g_min = 230;
	b_min = 130;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_yellow = PointCloudLab::check_color(cloud ,r_max,g_max,b_max,r_min,g_min,b_min);
	if (cloud_yellow->points.size() != 0)
		{
		cloud_yellow = remove_outliers(cloud_yellow);
		dimensions_yellow = check_block_dimensions(cloud_yellow);
		filtered_pub_yellow_.publish(cloud_yellow);
		}

	

	if (dimensions_red != "")
		str = str + "red block:" + dimensions_red + ";";
	if (dimensions_green != "")
		str = str + "green block:" + dimensions_green + ";";
	if (dimensions_blue != "")
		str = str + "blue block:" + dimensions_blue + ";";
	if (dimensions_orange != "")
		str = str + "orange block:" + dimensions_orange + ";";
	if (dimensions_yellow != "")
		str = str + "yellow block:" + dimensions_yellow + ";";

    	std_msgs::String msg;
    	msg.data = str;
	output_string = msg; 

	
    	//ros::spinOnce();
	//ROS_INFO_STREAM("[block detection] : " << output_string.data << std::endl);
	
	
  }

  


  // Proccess the point clouds
  void cloudCallback( const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    //ROS_INFO_STREAM("Recieved callback");

    block_poses_.poses.clear();
    // Basic point cloud conversions 

    // Convert from ROS to PCL
    pcl::PointCloud<pcl::PointXYZRGB> cloud = PointCloudLab::ros_msg_to_pcl(msg);

   // Transform to whatever frame we're working in, probably the arm's base frame, ie "base_link"
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed = 	PointCloudLab::transform_to_baxters_axis(cloud);
    if (cloud_transformed->points.size() == 0)
		return ;

    
    // Limit to things we think are roughly at the table height
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredZ = PointCloudLab::limit_points_to_table(cloud_transformed);

    
    // Perform the actual filtering ; downgrade the samples
    cloud_filteredZ = PointCloudLab::down_grade_samples(cloud_filteredZ);
    

    // publish the filtered output into the publisher semifiltered_pub_ defined as a global variable
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = cloud_filteredZ;
    semifiltered_pub_.publish(cloud_filtered);

   

    // Check if any points remain
    PointCloudLab::check_if_any_point_exists(cloud_filtered);

    // extract table surface (planar model) from the cloud
    cloud_filtered = PointCloudLab::plane_segmentation(cloud_filtered);

    // extract color
    PointCloudLab::extract_colors(cloud_filtered);
    
    
    // extract the colored clusters
    //std::vector <pcl::PointIndices> clsuter_indices = PointCloudLab::colored_clusters(cloud_filtered);
   
   
    
}

  std::string addBlock(float x, float y, float z, float angle)
  {

   	/*
	input : the point cloud
	output : return the coordinates of the block,
	functionality: get the x,y,z and angle of the blocks
		       find the orientation and return it as string
    	*/
	
   
    	//offsets
	//x = x - 0.03;
	//y = y + 0.151;
	//z= z + 0.20;
	x = x - 0.01 ;
	y = y + 0.04;
	z = z + 0.19;
    	geometry_msgs::Pose block_pose;
    	block_pose.position.x = x;
    	block_pose.position.y = y;
    	block_pose.position.z = z;

    	Eigen::Quaternionf quat(Eigen::AngleAxis<float>(angle, Eigen::Vector3f(0,0,1)));

    	block_pose.orientation.x = quat.x();
    	block_pose.orientation.y = quat.y();
    	block_pose.orientation.z = quat.z();
    	block_pose.orientation.w = quat.w();

    	// Discard noise
    	if( block_pose.position.y > 10 || block_pose.position.y < -10 )
    	{
      	//ROS_WARN_STREAM("Rejected block: " << block_pose );
    	}

    	//ROS_INFO_STREAM("Added block: \n" << block_pose );

    	//block_poses_.poses.push_back(block_pose);

    	//std::string dimensions = "";
    	std::ostringstream ostr,ostr1,ostr2;
    	ostr << x;
    	std::string x_s = ostr.str();
    	ostr1 << y;
    	std::string y_s = ostr1.str();
    	ostr2 << z;
    	std::string z_s = ostr2.str();
    	std::string dimensions = x_s + "," + y_s + "," +  z_s ;

    	return dimensions;
	
  }

};

};

int main(int argc, char** argv)
{
  

  ROS_INFO_STREAM("Starting Point Cloud Lab node");

  ros::init(argc, argv, "point_cloud_lab");
  ros::NodeHandle n;

  point_cloud_lab::PointCloudLab detector("pcl_lab" );

  ros::Publisher pub = n.advertise<std_msgs::String>("obj_pos", 1);
  //ros::Rate loop_rate(10);
  //ros::spinOnce();
  int count = 0;
  while (ros::ok())
  {
    if(output_string.data != ""){
    pub.publish(output_string);
    ROS_INFO_STREAM("[block detection] : " << output_string.data << std::endl);
    output_string.data = "";
    }
    ros::spinOnce();
    
  }


  ros::spin();
  return 0;
}

