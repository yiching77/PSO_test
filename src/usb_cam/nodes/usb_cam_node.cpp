/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
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
*   * Neither the name of the Robert Bosch nor the names of its
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
*
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include "tku_msgs/camera.h"
#include "tku_msgs/CameraInfo.h"
#include "tdataunit/tdataunit.hpp"

namespace usb_cam {

class UsbCamNode
{
private:
  ros::Subscriber getpara_sub_;
  ros::ServiceServer Camera_service;
  ros::NodeHandle node_;

  TdataUnit CameraSet;//for save camera parameters
public:
  // private ROS node handle
  //ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, auto_exposure_, auto_white_balance_, auto_Backlight_Compensation_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;



  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  void loadCameraFile(){
    CameraSet.LoadCameraSetFile();
    brightness_         = CameraSet.CameraParameterValue->brightness;
    contrast_           = CameraSet.CameraParameterValue->contrast;
    saturation_         = CameraSet.CameraParameterValue->saturation;
    white_balance_      = CameraSet.CameraParameterValue->white_balance;
    auto_white_balance_ = (CameraSet.CameraParameterValue->auto_white_balance==1)?true:false;
    auto_exposure_      = (CameraSet.CameraParameterValue->auto_exposure==1)?true:false;
    auto_Backlight_Compensation_ = (CameraSet.CameraParameterValue->auto_Backlight_Compensation==1)?true:false;
  } 
  
  void saveCameraFile(){
    
    ROS_INFO("brightness                        = %d",brightness_);
    ROS_INFO("contrast                          = %d",contrast_);
    ROS_INFO("saturation                        = %d",saturation_);
    ROS_INFO("white_balance                     = %d",white_balance_);
    ROS_INFO("auto_white_balance                = %d",(auto_white_balance_==true)?1:0);
    ROS_INFO("auto_exposure                     = %d",(auto_exposure_==true)?1:0);
    ROS_INFO("auto_Backlight_Compensation       = %d",(auto_Backlight_Compensation_==true)?1:0);
    ROS_INFO("===================================");
    CameraSet.CameraParameterValue->brightness         = brightness_;
    CameraSet.CameraParameterValue->contrast           = contrast_;
    CameraSet.CameraParameterValue->saturation         = saturation_;
    CameraSet.CameraParameterValue->white_balance      = white_balance_;
    CameraSet.CameraParameterValue->auto_exposure       = (auto_exposure_==1)?true:false;
    CameraSet.CameraParameterValue->auto_white_balance = (auto_white_balance_==1)?true:false;
    CameraSet.CameraParameterValue->auto_Backlight_Compensation = (auto_Backlight_Compensation_==1)?true:false;
    CameraSet.SaveCameraSetFile();
  }

  void SetCameraParameter(){
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (white_balance_ >= 0)
    {
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }
  } 

  void getparaCallback(const tku_msgs::camera &msg)
  {
    brightness_         = int(msg.brightness);
    contrast_           = int(msg.contrast);
    saturation_         = int(msg.saturation);
    white_balance_      = int(msg.white_balance);
    auto_white_balance_ = msg.auto_white_balance;
    auto_exposure_       = msg.auto_exposure; 
    auto_Backlight_Compensation_ = msg.auto_Backlight_Compensation;


    // check auto exposure
    if (!auto_exposure_)
    {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
    }else{
      cam_.set_v4l_parameter("exposure_auto", 3);
    }

    if (brightness_ >= -64)
    {
      cam_.set_v4l_parameter("brightness", (brightness_+70)*2);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_*2);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_*2);
    }

    if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }
    saveCameraFile();
  }
  
  bool CallCameraInfoFunction(tku_msgs::CameraInfo::Request &req, tku_msgs::CameraInfo::Response &res)
  {
    res.brightness         = brightness_;
    res.contrast           = contrast_;
    res.saturation         = saturation_;
    res.white_balance      = white_balance_;
    res.auto_white_balance = auto_white_balance_;
    res.auto_exposure       = auto_exposure_;
    res.auto_Backlight_Compensation = auto_Backlight_Compensation_;
  }

  UsbCamNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);
    getpara_sub_ = node_.subscribe("/Camera_Topic", 1, &UsbCamNode::getparaCallback,this);
    Camera_service = node_.advertiseService("/CameraInfo", &UsbCamNode::CallCameraInfoFunction,this);

    //load CameraSet.ini to get parameters
    loadCameraFile();

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, brightness_); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, contrast_); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, saturation_); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", auto_exposure_, auto_exposure_);
    node_.param("exposure", exposure_, 20);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, auto_white_balance_);
    node_.param("white_balance", white_balance_, white_balance_);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    // check for default camera info
    //if (!cinfo_->isCalibrated()) //if usb_cam package has error, you can try uncomment it first. It is default code.
    //{
    cinfo_->setCameraName(video_device_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = img_.header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    cinfo_->setCameraInfo(camera_info);
    //}


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (sharpness_ >= 0)
    {
      cam_.set_v4l_parameter("sharpness", sharpness_);
    }

    if (gain_ >= 0)
    {
      cam_.set_v4l_parameter("gain", gain_);
    }

    // check auto white balance
   if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check auto exposure
    if (!auto_exposure_)
    {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }

    // check auto focus
    if (autofocus_)
    {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
      {
        cam_.set_v4l_parameter("focus_absolute", focus_);
      }
    }
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);
    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    loadCameraFile();
    SetCameraParameter();
    while (node_.ok()) 
    {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();

    }
    // Save the camera parameter to CameraSet.ini
    return true;
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
