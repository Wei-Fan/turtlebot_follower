/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
#include "turtlebot_follower/FollowerConfig.h"

#include <depth_image_proc/depth_traits.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

int p0_x=-1,p0_y=-1,p1_x=-1,p1_y=-1;
int roi_width;
int roi_height;
int target_x, target_y;

namespace turtlebot_follower
{

//* The turtlebot follower nodelet.
/**
 * The turtlebot follower nodelet. Subscribes to point clouds
 * from the 3dsensor, processes them, and publishes command vel
 * messages.
 */
class TurtlebotFollower : public nodelet::Nodelet
{
public:
  /*!
   * @brief The constructor for the follower.
   * Constructor for the follower.
   */
  TurtlebotFollower() : min_y_(0.1), max_y_(0.5),
                        min_x_(-0.2), max_x_(0.2),
                        max_z_(0.8), goal_z_(0.6),
                        z_scale_(1.0), x_scale_(5.0),
                        isFirst(true),recieve(false),
                        // p0_x(-1),p0_y(-1),p1_x(-1),p1_y(-1),
                        shift_scale(0.0001)
  {
    // namedWindow("normal",WINDOW_AUTOSIZE);
    // isFirst = true;

  }

  ~TurtlebotFollower()
  {
    // destoryWindow("normal");
    // imshow("normal",src_img);
    // waitKey(0);
    delete config_srv_;
  }

private:
  double min_y_; /**< The minimum y position of the points in the box. */
  double max_y_; /**< The maximum y position of the points in the box. */
  double min_x_; /**< The minimum x position of the points in the box. */
  double max_x_; /**< The maximum x position of the points in the box. */
  double max_z_; /**< The maximum z position of the points in the box. */
  double goal_z_; /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  bool   enabled_; /**< Enable/disable following; just prevents motor commands */

  bool isFirst;
  bool recieve;
  
  float shift_scale;
  Mat target;
  Mat src_depth_img;
  Mat src_rgb_img;
  Mat mask;
  Mat src_img;
  // Service for start/stop following
  ros::ServiceServer switch_srv_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>* config_srv_;

  /*!
   * @brief OnInit method from node handle.
   * OnInit method from node handle. Sets up the parameters
   * and topics.
   */
  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    private_nh.getParam("min_y", min_y_);
    private_nh.getParam("max_y", max_y_);
    private_nh.getParam("min_x", min_x_);
    private_nh.getParam("max_x", max_x_);
    private_nh.getParam("max_z", max_z_);
    private_nh.getParam("goal_z", goal_z_);
    private_nh.getParam("z_scale", z_scale_);
    private_nh.getParam("x_scale", x_scale_);
    private_nh.getParam("enabled", enabled_);

    cmdpub_ = private_nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    markerpub_ = private_nh.advertise<visualization_msgs::Marker>("marker",1);
    bboxpub_ = private_nh.advertise<visualization_msgs::Marker>("bbox",1);
    sub_= nh.subscribe<sensor_msgs::Image>("depth/image_rect", 1, &TurtlebotFollower::imagecb, this);

    sub_depth_ = nh.subscribe<sensor_msgs::Image>("depth/image_rect", 1, &TurtlebotFollower::depth_imagecb, this);
    sub_rgb_ = nh.subscribe<sensor_msgs::Image>("rgb/image_rect_color", 1, &TurtlebotFollower::rgb_imagecb, this);

    switch_srv_ = private_nh.advertiseService("change_state", &TurtlebotFollower::changeModeSrvCb, this);

    config_srv_ = new dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>(private_nh);
    dynamic_reconfigure::Server<turtlebot_follower::FollowerConfig>::CallbackType f =
        boost::bind(&TurtlebotFollower::reconfigure, this, _1, _2);
    config_srv_->setCallback(f);

    ros::NodeHandle node;
    ros::Timer timer = node.createTimer(ros::Duration(0.02), &TurtlebotFollower::iteration,this);
    ros::spin();
  }

  void reconfigure(turtlebot_follower::FollowerConfig &config, uint32_t level)
  {
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    max_z_ = config.max_z;
    goal_z_ = config.goal_z;
    z_scale_ = config.z_scale;
    x_scale_ = config.x_scale;
  }

  /*!
   *image callbacks and mouse function
   */
  void depth_imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    // sensor_msgs::Image img = *depth_msg;
    cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    src_depth_img = cv_ptr->image;
    
    Mat tmp_mask = Mat(src_depth_img != src_depth_img);
    src_depth_img.setTo(10,tmp_mask);
    mask = Mat::zeros(src_depth_img.size(),CV_8UC1);

    blur(src_depth_img, src_depth_img, Size(3,3));

    int rows = src_depth_img.rows;
    int cols = src_depth_img.cols;

    for (int i = 0; i < rows; ++i)
    {
      for (int j = 0; j < cols; ++j)
      {
        float t = src_depth_img.at<float>(i,j)*255/10;        
        if (t > 80)
          // mask.at<uchar>(i,j) = 255;
          mask.at<uchar>(i,j) = 255;
        else
          // mask.at<uchar>(i,j) = uchar(t);
          mask.at<uchar>(i,j) = 0;
      }
    }
    // imshow("mask",mask);
    // waitKey(0);
    // imwrite("src/restart/src/test.png",tmp_img);//test alright
    // ROS_INFO("~~~ max : %f ~~~ min : %f",max, min);
  }

  void rgb_imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    // sensor_msgs::Image img = *depth_msg;
    cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::BGR8);
    src_rgb_img = cv_ptr->image;
    
    
    // imwrite("src/restart/src/pm_test.png",src_rgb_img);//test alright
    // ROS_INFO("~~~ max : %f ~~~ min : %f",max, min);
    recieve = true;
  }

  static void onMouse(int event, int x, int y, int, void* userInput)
  {
    if (event != EVENT_LBUTTONDOWN && event != EVENT_LBUTTONUP) return;
    //printf("###########onMouse x : %d\n", x);
    //printf("###########onMouse y : %d\n", y);
    
    Mat *img = (Mat*)userInput;
    if (event == EVENT_LBUTTONDOWN)
    {
      p0_x = x;
      p0_y = y;
    } else if (event == EVENT_LBUTTONUP)
    {
      p1_x = x;
      p1_y = y;
      roi_width = abs(p1_x - p0_x);
      roi_height = abs(p1_y - p0_y);
      target_x = (p0_x+p1_x)/2;
      target_y = (p0_y+p1_y)/2;
      rectangle(*img,Rect(p0_x,p0_y,roi_width,roi_height),Scalar(0,0,255),1,1,0);
      imshow("view",*img);
    }
  }

  /*!
   *image processing lies in iteration
   *melting both rgb image and depth image
   *Publishes cmd_vel messages with the goal from the image
   */
  void iteration(const ros::TimerEvent& e)
  {
    // ROS_INFO("iteration~~~~");
    if (!recieve)
      return;
    if (isFirst)
    {
      isFirst = false;
      Mat src_rgb_img_0 = src_rgb_img;
      bool init_done = false;
      while(!init_done)
      {
        namedWindow("view");
        imshow("view",src_rgb_img_0);
        setMouseCallback("view", onMouse, &src_rgb_img_0);
        waitKey(0);
        destroyWindow("view");
        if (p1_x>0)
        {
          init_done = true;
        }
      }
      ROS_INFO("init done ~~~");
      Mat src_rgb_roi = src_rgb_img(Rect(p0_x,p0_y,roi_width,roi_height));
      // imshow("roi",src_rgb_roi);
      // waitKey(0);
      // destroyWindow("roi");
      Mat src_hsv_img;
      vector<Mat> hsv_vec;
      cvtColor(src_rgb_roi,src_hsv_img,CV_BGR2HSV_FULL);
      split(src_hsv_img,hsv_vec);
      target = hsv_vec[0];

      // imshow("H",img_h);
      // waitKey(0);
      // destroyWindow("H");
      // // img_h.convertTo(img_h,CV_32F);

      // ROS_INFO("convert done ~~~");
      // int mean = 0;
      // for (int i = 0; i < img_h.rows; ++i)
      // {
      //  for (int j = 0; j < img_h.cols; ++j)
      //   {
      //    // ROS_INFO("%d, %d : %d",i,j,img_h.at<uchar>(i,j));
      //    mean += img_h.at<uchar>(i,j);
      //   } 
      // }
      // mean = int(mean/(roi_width*roi_height));
      // ROS_INFO("mean : %d",mean);
      // destroyWindow("view");
    } else {
      /*fitering*/
      src_rgb_img.setTo((0,0,0),mask);
      // imshow("after mask",src_rgb_img);
      // waitKey(0);
      // destroyWindow("after mask");
      bool stop = false;
      while(!stop)
      {
        /*obtain roi in HSV form*/
        Mat src_rgb_roi = src_rgb_img(Rect(target_x-roi_width/2,target_y-roi_height/2,roi_width,roi_height));
        Mat src_hsv_img, img_h;
        vector<Mat> hsv_vec;
        cvtColor(src_rgb_roi,src_hsv_img,CV_BGR2HSV_FULL);
        split(src_hsv_img,hsv_vec);
        img_h = hsv_vec[0];

        /*calculate the shift vector*/
        float shift_x=0.0, shift_y=0.0;
        for (int i = 0; i < img_h.rows; ++i)
        {
          for (int j = 0; j < img_h.cols; ++j)
           {
            shift_x += (i-target_x)*(255-abs(img_h.at<uchar>(i,j)-target.at<uchar>(i,j)))*shift_scale;
            shift_y += (j-target_y)*(255-abs(img_h.at<uchar>(i,j)-target.at<uchar>(i,j)))*shift_scale;
           } 
        }
        ROS_INFO("need to tune the shift_scale ~~~ shift_x : %f ~~~ shift_y : %f", shift_x, shift_y);
        if (fabs(shift_x) < 5 && fabs(shift_y) < 5)
        {
          stop = true;
        } else {
          bool x_stop = false, y_stop = false;
          if (target_x+shift_x-roi_width/2>0 && target_x+shift_x-roi_width/2<src_rgb_img.rows)
          {
            target_x += shift_x;
          } else {
            x_stop = true;
          }
          if (target_y+shift_y-roi_height/2>0 && target_y+shift_y-roi_height/2<src_rgb_img.cols)
          {
            target_y += shift_y;
          } else {
            y_stop = true;
          }
          if (x_stop && y_stop)
          {
            stop = true;
          }
        }
      }
      rectangle(src_rgb_img,Rect(target_x-roi_width/2,target_y-roi_height/2,roi_width,roi_height),Scalar(0,0,255),1,1,0);
      imshow("monitor",src_rgb_img);
      waitKey(0);
      destroyWindow("monitor");
    }
  }
  /*!
   * @brief Callback for point clouds.
   * Callback for depth images. Use a front view filter
   * Callback for rgb images .
   * @param cloud The point cloud message.
   */
  void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    // sensor_msgs::Image img = *depth_msg;
    cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    src_img = cv_ptr->image;
    Mat mask = Mat(src_img != src_img);
    src_img.setTo(0,mask);//after wipping NAN, max = 10.000 min = 0.000
    
    // ROS_INFO("image info : %d, %d", src_img.rows, src_img.cols);

    // imshow("normal",src_img);

    // Precompute the sin function for each row and column
    uint32_t image_width = depth_msg->width;
    float x_radians_per_pixel = 60.0/57.0/image_width;
    float sin_pixel_x[image_width];
    for (int x = 0; x < image_width; ++x) {
      sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
    }

    uint32_t image_height = depth_msg->height;
    float y_radians_per_pixel = 45.0/57.0/image_width;
    float sin_pixel_y[image_height];
    for (int y = 0; y < image_height; ++y) {
      // Sign opposite x for y up values
      sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
    }

    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //Number of points observed
    unsigned int n = 0;

    //Iterate through all the points in the region and find the average of the position
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(float);
    for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
    {
     for (int u = 0; u < (int)depth_msg->width; ++u)
     {
       float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
       if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
       float y_val = sin_pixel_y[v] * depth;
       float x_val = sin_pixel_x[u] * depth;
       if ( y_val > min_y_ && y_val < max_y_ &&
            x_val > min_x_ && x_val < max_x_)
       {
         x += x_val;
         y += y_val;
         z = std::min(z, depth); //approximate depth as forward.
         n++;
       }
     }
    }

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    if (n>4000)
    {
      x /= n;
      y /= n;
      if(z > max_z_){
        ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot", z);
        if (enabled_)
        {
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
        return;
      }

      ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);
      publishMarker(x, y, z);

      if (enabled_)
      {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmd->linear.x = (z - goal_z_) * z_scale_;
        cmd->angular.z = -x * x_scale_;
        cmdpub_.publish(cmd);
      }
    }
    else
    {
      ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);
      publishMarker(x, y, z);

      if (enabled_)
      {
        cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      }
    }

    publishBbox();
  }

  bool changeModeSrvCb(turtlebot_msgs::SetFollowState::Request& request,
                       turtlebot_msgs::SetFollowState::Response& response)
  {
    if ((enabled_ == true) && (request.state == request.STOPPED))
    {
      ROS_INFO("Change mode service request: following stopped");
      cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      enabled_ = false;
    }
    else if ((enabled_ == false) && (request.state == request.FOLLOW))
    {
      ROS_INFO("Change mode service request: following (re)started");
      enabled_ = true;
    }

    response.result = response.OK;
    return true;
  }

  void publishMarker(double x,double y,double z)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    markerpub_.publish( marker );
  }

  void publishBbox()
  {
    double x = (min_x_ + max_x_)/2;
    double y = (min_y_ + max_y_)/2;
    double z = (0 + max_z_)/2;

    double scale_x = (max_x_ - x)*2;
    double scale_y = (max_y_ - y)*2;
    double scale_z = (max_z_ - z)*2;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera_rgb_optical_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = -y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    bboxpub_.publish( marker );
  }

  ros::Subscriber sub_,sub_depth_,sub_rgb_;
  ros::Publisher cmdpub_;
  ros::Publisher markerpub_;
  ros::Publisher bboxpub_;
};

PLUGINLIB_EXPORT_CLASS(turtlebot_follower::TurtlebotFollower, nodelet::Nodelet)

}
