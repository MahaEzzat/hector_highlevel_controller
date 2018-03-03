#ifndef HECTORHIGHLEVELCONTROLLER_H_
#define HECTORHIGHLEVELCONTROLLER_H_

#include <ros/ros.h>
#include <string.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <math.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
//Computer Vision
#include <cv_bridge/cv_bridge.h>
#include <opencv-3.3.1/opencv2/imgproc/imgproc.hpp>
#include <opencv-3.3.1/opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>

namespace hector_highlevel_controller
{
	class HectorHighlevelController
	{
		public:
			// constructor
			HectorHighlevelController(ros::NodeHandle& n);
			
			// destructor
			virtual ~HectorHighlevelController();
		private:

		//create Ros nodehandle, sub, and pub 
			ros::NodeHandle& n_;
			ros::Subscriber sub_;
			ros::Subscriber sub_cam;
			ros::Subscriber sub_IMU;
			ros::Subscriber sub_pos;
			ros::Publisher  pub_;
		//	ros::Publisher  pub_cam;
			ros::Publisher  pub_mark;
		    ros::Subscriber sub;
            ros::Publisher pub;
			
			
		
			
		//create methods	
			bool Parameters();
			void twistmeth();
			void mark();
     
	  void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	  void camCallback(const sensor_msgs::Image::ConstPtr& msg_cam);
	  void imuCallback(const sensor_msgs::Imu &msg3);	
	  void posCallback(const geometry_msgs::PoseStamped &msg4);
	  void PID(); 
	  void Path();
 

          //create arguments
          visualization_msgs::Marker marker;
		  geometry_msgs::Twist msg2;
          laser_geometry::LaserProjection projector_;
          tf::TransformListener tf_listener;
    
		  
			float ref_angle;
			float smallest_distance;
			float piller_x,piller_y,piller_z;
			double roll,yaw,pitch;
			float roll_dot,yaw_dot,pitch_dot;
			float vx,vy,vz;
			float vxi,vyi,vzi;
			float pos_x,pos_y,pos_z; 
			float pos_prev_x,pos_prev_y,pos_prev_z;
			float error_x,error_y,error_z,error;
			float error_d_x,error_d_y,error_d_z;
			float error_i_x,error_i_y,error_i_z;
			float t,t_prev;
			float pos_targ_x;
			float pos_targ_y;
			float pos_targ_z;
			float kp_x,kp_y,kp_z;
			float ki_x,ki_y,ki_z;
			float kd_x,kd_y,kd_z;
			double x,y,r;
			double dt;
			int count_path=0;
			int count=0;
			double theta=0.0;
            std::vector< double > pathx;
            std::vector< double > pathy;
		    std::vector< double > centriod;
            std::vector< double > rgb;
			  //(centriod #1)(rgb #2)

			
			 
			 
	};
	
}




#endif