#include <ros/ros.h>
#include "hector_highlevel_controller/HectorHighlevelController.h"

namespace hector_highlevel_controller
{
    HectorHighlevelController::HectorHighlevelController(ros::NodeHandle& n) :
    n_(n)
    {

if(!Parameters())
        {
            ROS_ERROR("Error in loading parameters");  
            ros::shutdown();
        }
     HectorHighlevelController::Path();
    
      sub_pos = n_.subscribe("/ground_truth_to_tf/pose",1,&HectorHighlevelController::posCallback,this);   
      sub_IMU = n_.subscribe("/raw_imu",1,&HectorHighlevelController::imuCallback,this);
      sub_ = n_.subscribe("/camera/depth/points",1,&HectorHighlevelController::scanCallback,this);  
      sub_cam = n_.subscribe("/camera/rgb/image_raw",1,&HectorHighlevelController::camCallback,this);  
      
      pub_=  n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);   
      pub_mark = n_.advertise<visualization_msgs::Marker>( "/visualization_marker", 1 );
      pub = n.advertise<sensor_msgs::PointCloud2> ("/cloud", 1);
      //pub_cam = n.advertise<sensor_msgs::Image> ("/Image", 1); 
    }

    HectorHighlevelController::~HectorHighlevelController()
    {

    }



//Methods  
    
  void HectorHighlevelController::scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
  if(!(msg == NULL))
{      

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  sensor_msgs::PointCloud2 output1;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*msg, *cloud);

  // Perform the actual filtering(downsampling)
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output,output_;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

 pcl_ros::transformPointCloud ("world", output, output_, tf_listener);

pcl::PointCloud< pcl::PointXYZRGB>::Ptr PointCloudXYZ(new pcl::PointCloud<pcl::PointXYZRGB>);


pcl::fromROSMsg(output_,*PointCloudXYZ);




//RANSAC
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()),cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::SACSegmentation<pcl::PointXYZRGB> seg;
seg.setOptimizeCoefficients (true);
seg.setModelType (pcl::SACMODEL_PLANE);
seg.setMethodType (pcl::SAC_RANSAC);
seg.setDistanceThreshold (0.1);
//seg.setInputCloud (PointCloudXYZ);
//seg.segment (*inliers, *coefficients);

//exclude inlier points in one segment

 int i=0, nr_points = (int) PointCloudXYZ->points.size (); //inital size
  while (PointCloudXYZ->points.size () > 0.5 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (PointCloudXYZ);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (PointCloudXYZ);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *PointCloudXYZ = *cloud_f;
  }

// Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_plane);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (600);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_plane);
  ec.extract (cluster_indices);

 long cluster_red=0,cluster_green=0,cluster_blue=0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    int j=0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      cluster_red=0,cluster_green=0,cluster_blue=0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

      cloud_cluster->points.push_back (cloud_plane->points[*pit]); //*
      uint8_t red = cloud_cluster->points[j].r;
          cloud_cluster->points[j].r=cloud_cluster->points[j].b;
          cloud_cluster->points[j].b=red;
          //cluster histogram
          cluster_red +=cloud_cluster->points[j].r;
          cluster_green +=cloud_cluster->points[j].g;
          cluster_blue +=cloud_cluster->points[j].b;
          j++;
                 }

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

j = cloud_cluster->width;
cluster_red /=j;
cluster_green /=j;
cluster_blue /=j;
//color scalling
uint8_t max_color=0;
if(cluster_red>=cluster_green && cluster_red>=cluster_blue) max_color=cluster_red;
else if (cluster_green>=cluster_red && cluster_green>=cluster_blue) max_color=cluster_green;
else if (cluster_blue>=cluster_red && cluster_blue>=cluster_green) max_color=cluster_blue;
cluster_red=(cluster_red/(double)max_color)*255;
cluster_green=(cluster_green/(double)max_color)*255;
cluster_blue=(cluster_blue/(double)max_color)*255;

//ROS_INFO("Cluster Color is: red [%d], green [%d], blue [%d]",cluster_red,cluster_green,cluster_blue);

 


    pcl::toROSMsg(*cloud_cluster,output1);
    output1.header.frame_id = "world";
  // output1_.header.frame_id = "/world";

//Centriod of the cluster
int cloudsize = (cloud_cluster->width) * (cloud_cluster->height);
int  sum_x=0,sum_y=0,sum_z=0,num=0;
for (int i=0; i< cloudsize; i++)
{
  sum_x +=cloud_cluster->points[i].x;
  sum_y +=cloud_cluster->points[i].y;
  sum_z +=cloud_cluster->points[i].z;
  num++;
   }
   
//marker   
piller_x = (double)(sum_x)/num;
piller_y = (double)(sum_y)/num;
piller_z = (double)(sum_z)/num;
    
    //output1.header.frame_id = "world";
     // Publish the data
     
         //ROS_INFO("piller_x: [%f]  piller_y: [%f] piller_z: [%f]",piller_x,piller_y,piller_z);

     }
      pub.publish (output1);
      centriod.push_back(piller_x);
      centriod.push_back(piller_y);
      centriod.push_back(piller_z);
      rgb.push_back(cluster_red);
      rgb.push_back(cluster_green);
      rgb.push_back(cluster_blue);
      //ROS_INFO("Cluster Color is: red [%d], green [%d], blue [%d]",centriod[0],cluster_green,cluster_blue);

   }
 }


 void HectorHighlevelController::imuCallback(const sensor_msgs::Imu &msg3)
    {

 tf::Quaternion bg(msg3.orientation.x,msg3.orientation.y,msg3.orientation.z,msg3.orientation.w);
 tf::Matrix3x3(bg).getRPY(roll,pitch,yaw);
//ROS_INFO("roll: [%f]  pitch: [%f]  yaw: [%f]",roll,pitch,yaw);
      HectorHighlevelController::PID();
        HectorHighlevelController::twistmeth();
         HectorHighlevelController::mark();
            pub_.publish(msg2);
            pub_mark.publish(marker);
    }


void HectorHighlevelController::twistmeth()
    {
                   msg2.linear.x = vxi;
                   msg2.linear.y = vyi;
                   msg2.linear.z = vzi;
                   msg2.angular.x = 0; 
                   msg2.angular.y = 0; 
                   msg2.angular.z = 0;
                   if(error<0.01) msg2.angular.z = 1;
                   }



void HectorHighlevelController::posCallback(const geometry_msgs::PoseStamped &msg4)
    {

  pos_x =  msg4.pose.position.x;
  pos_y =  msg4.pose.position.y;
  pos_z =  msg4.pose.position.z;

//ROS_INFO("x: [%f]  y: [%f]  z: [%f]",pos_x,pos_y,pos_z);

  }

void HectorHighlevelController::PID()
{

pos_targ_x = pathx[count_path];
pos_targ_y = pathy[count_path];
pos_targ_z = 6.5;


if(count==0)
{
t_prev = ros::Time::now().toSec();
  error_d_x =0;
  error_d_y =0;
  error_d_z =0;
  error_i_x = 0;
  error_i_y = 0;
  error_i_z = 0;
  error=0.6;
  count++;
}

else
{ 
t = ros::Time::now().toSec() ;
dt = t - t_prev;
t_prev = t;

error_d_x = (pos_prev_x-pos_x)/dt;
error_d_y = (pos_prev_y-pos_y)/dt;
error_d_z = (pos_prev_z-pos_z)/dt;

error_i_x += error_x*dt;
error_i_y += error_y*dt;
error_i_z += error_z*dt;
}

error_x = pos_targ_x-pos_x;
error_y = pos_targ_y-pos_y;
error_z = pos_targ_z-pos_z;
error = sqrt(error_x*error_x+error_y*error_y+error_z*error_z);

//if(count_path<2){
vx = kp_x*error_x + kd_x *error_d_x+error_i_x*ki_x;
vy = kp_y*error_y + kd_y *error_d_y+error_i_y*ki_z;;
vz = kp_z*error_z + kd_z *error_d_z+error_i_z*ki_y;;

if( error < 0.1) {
    count_path++;
    if(count_path>(pathx.size()-1)) count_path=0;
}

/* Velocity Profile
}
else{
    vx = (pathx[count_path] - pathx[(count_path-1)])/dt;
    vy = (pathy[count_path] - pathy[(count_path-1)])/dt;
    vz = 0.0;
     count_path++;
     if(count_path>(pathx.size()-1)) count_path=0;
} */
//ROS_INFO("vx: [%f]  vy: [%f]  vz: [%f]",vx,vy,vz);


pos_prev_x = pos_x;
pos_prev_y = pos_y;
pos_prev_z = pos_z;



vxi = (cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch))*vx - cos(roll)*sin(yaw)*vy + (cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw))*vz;
vyi = (sin(yaw)*cos(pitch)+sin(roll)*cos(yaw)*sin(pitch))*vx + cos(roll)*cos(yaw)*vy + (sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw))*vz;
vzi = -cos(roll)*sin(pitch)*vx  +  sin(roll)*vy   + cos(pitch)*cos(roll)*vz;


}


void HectorHighlevelController::Path()
{


 /*Infinity figure
  while(theta<44/7){
    theta += 0.2;
    r=5+5*cos(2*theta);
    x = r*cos(theta);
    y = r*sin(theta);
    pathx.push_back(x);
    pathy.push_back(y); 
} */

// Object_position
/* pathx.push_back(-0.1);
 pathy.push_back(-0.1);
 pathx.push_back(-0.1);
 pathy.push_back(-0.1); */

 //circule    
  pathx.push_back(0.0);
  pathy.push_back(0.0);
  while(theta<44/7){
    theta += 0.5;
    r=2;
    x = r*cos(theta);
    y = r*sin(theta);
    pathx.push_back(x);
    pathy.push_back(y); 
} 

 

}



    void HectorHighlevelController::mark()
  {
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "Piller";
    
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

   marker.pose.position.x = piller_x;
   marker.pose.position.y = piller_y;
   marker.pose.position.z = piller_z;
   marker.scale.x = 1;
   marker.scale.y = 1;
   marker.scale.z = 0.1;
   marker.color.a = 1.0; // Don't forget to set the alpha!
   marker.color.r = 0.0;
   marker.color.g = 1.0;
   marker.color.b = 0.0;  
   
  } 



  bool HectorHighlevelController::Parameters()
    {
        if(!n_.getParam("kp_x_",kp_x)) return false;
        if(!n_.getParam("kp_y_",kp_y)) return false;
        if(!n_.getParam("kp_z_",kp_z)) return false;
        if(!n_.getParam("ki_x_",ki_x)) return false;
        if(!n_.getParam("ki_y_",ki_y)) return false;
        if(!n_.getParam("ki_z_",ki_z)) return false;
        if(!n_.getParam("kd_x_",kd_x)) return false;
        if(!n_.getParam("kd_y_",kd_y)) return false;
        if(!n_.getParam("kd_z_",kd_z)) return false;


        return true;
    }
    void HectorHighlevelController::camCallback(const sensor_msgs::Image::ConstPtr& msg_cam)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg_cam, sensor_msgs::image_encodings::RGBA8);
      //pub_cam.publish(cv_ptr);
    }

}
