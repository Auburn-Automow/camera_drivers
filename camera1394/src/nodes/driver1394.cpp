// $Id: driver1394.cpp 36902 2011-05-26 23:20:18Z joq $

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2009, 2010 Jack O'Quin, Patrick Beeson
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
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
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

#include <boost/format.hpp>

#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>

#include "driver1394.h"
#include "camera1394/Camera1394Config.h"
#include "features.h"

/** @file

@brief ROS driver for IIDC-compatible IEEE 1394 digital cameras.

This is a ROS driver for 1394 cameras, using libdc1394.  It can be
instantiated as either a node or a nodelet.  It is written with with
minimal dependencies, intended to fill a role in the ROS image
pipeline similar to the other ROS camera drivers.

@par Advertises (monocular mode, default)

 - @b camera/image_raw topic (sensor_msgs/Image) raw 2D camera images

 - @b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
   information for each image.

@par Advertises (stereo mode, default)

 - @b camera/left/image_raw topic (sensor_msgs/Image) raw 2D camera images
 - @b camera/left/camera_info topic (sensor_msgs/CameraInfo) calibration

 - @b camera/right/image_raw topic (sensor_msgs/Image) raw 2D camera images
 - @b camera/right/camera_info topic (sensor_msgs/CameraInfo) calibration

*/

namespace camera1394_driver
{
  // some convenience typedefs
  typedef camera1394::Camera1394Config Config;
  typedef driver_base::Driver Driver;
  typedef driver_base::SensorLevels Levels;

  Camera1394Driver::Camera1394Driver(ros::NodeHandle priv_nh,
                                     ros::NodeHandle camera_nh):
    state_(Driver::CLOSED),
    reconfiguring_(false),
    priv_nh_(priv_nh),
    camera_nh_(camera_nh),
    left_camera_nh_(camera_nh, "left"),
    right_camera_nh_(camera_nh, "right"),
    camera_name_("camera"),
    dev_(new camera1394::Camera1394()),
    srv_(priv_nh),
    cycle_(1.0)                        // slow poll when closed
  {}

  Camera1394Driver::~Camera1394Driver()
  {}

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void Camera1394Driver::closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        dev_->close();
        state_ = Driver::CLOSED;
      }
  }

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   */
  bool Camera1394Driver::openCamera(Config &newconfig)
  {
    bool success = false;
    int retries = 2;                    // number of retries, if open fails
    do
      {
        try
          {
            if (0 == dev_->open(newconfig))
              {
                if (camera_name_ != dev_->device_id_)
                  {
                    camera_name_ = dev_->device_id_;
                    if (!left_cinfo_->setCameraName(camera_name_))
                      {
                        // GUID is 16 hex digits, which should be valid.
                        // If not, use it for log messages anyway.
                        ROS_WARN_STREAM("[" << camera_name_
                                        << "] name not valid"
                                        << " for camera_info_manger");
                      }
                    else if (newconfig.multicam != "mono")
                    {
                      right_cinfo_->setCameraName(camera_name_);
                    }
                  }
                ROS_INFO_STREAM("[" << camera_name_ << "] opened: "
                                << newconfig.video_mode << ", "
                                << newconfig.frame_rate << " fps, "
                                << newconfig.iso_speed << " Mb/s");
                state_ = Driver::OPENED;
                left_calibration_matches_ = true;
                right_calibration_matches_ = true;
                success = true;
              }

          }
        catch (camera1394::Exception& e)
          {
            state_ = Driver::CLOSED;    // since the open() failed
            if (retries > 0)
              ROS_WARN_STREAM("[" << camera_name_
                              << "] exception opening device (retrying): "
                              << e.what());
            else
              ROS_ERROR_STREAM("[" << camera_name_
                               << "] device open failed: " << e.what());
          }
      }
    while (!success && --retries >= 0);

    return success;
  }


  /** device poll */
  void Camera1394Driver::poll(void)
  {
    // Do not run concurrently with reconfig().
    //
    // The mutex lock should be sufficient, but the Linux pthreads
    // implementation does not guarantee fairness, and the reconfig()
    // callback thread generally suffers from lock starvation for many
    // seconds before getting to run.  So, we avoid acquiring the lock
    // if there is a reconfig() pending.
    bool do_sleep = true;
    if (!reconfiguring_)
      {
        boost::mutex::scoped_lock lock(mutex_);
        do_sleep = (state_ == Driver::CLOSED);
        if (!do_sleep)
          {
            // driver is open, read the next image still holding lock
            sensor_msgs::ImagePtr left_image(new sensor_msgs::Image);
            sensor_msgs::ImagePtr right_image(new sensor_msgs::Image);
            if (read(left_image, right_image))
              {
                publish(left_image, right_image);
              }
          }
      }

    if (do_sleep)
      {
        // device was closed or poll is not running, sleeping avoids
        // busy wait (DO NOT hold the lock while sleeping)
        cycle_.sleep();
      }
  }

  /** Publish camera stream topics
   *
   *  @param image points to latest camera frame
   */
  void Camera1394Driver::publish(const sensor_msgs::ImagePtr &left_image,
                             const sensor_msgs::ImagePtr &right_image)
  {
  if (config_.multicam == "mono")
  {
    left_image->header.frame_id = config_.frame_id;
    sensor_msgs::CameraInfoPtr
      ci(new sensor_msgs::CameraInfo(left_cinfo_->getCameraInfo()));
    checkCameraInfo(left_image, ci, left_calibration_matches_, camera_name_);
    dev_->setOperationalParameters(*ci);

    ci->header.frame_id = config_.frame_id;
    ci->header.stamp = left_image->header.stamp;

    left_image_pub_.publish(left_image, ci);
  }
  else
  {
    sensor_msgs::CameraInfoPtr
      left_ci(new sensor_msgs::CameraInfo(left_cinfo_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr
      right_ci(new sensor_msgs::CameraInfo(right_cinfo_->getCameraInfo()));
    checkCameraInfo(left_image, left_ci, left_calibration_matches_, camera_name_ + "/left");
    checkCameraInfo(right_image, right_ci, right_calibration_matches_, camera_name_ + "/right");

    dev_->setOperationalParameters(*left_ci);
    dev_->setOperationalParameters(*right_ci);

        left_ci->header.frame_id = config_.frame_id;
        left_ci->header.stamp = left_image->header.stamp;
        right_ci->header.frame_id = config_.frame_id;
        right_ci->header.stamp = right_image->header.stamp;

        // Publish via image_transport
        left_image_pub_.publish(left_image, left_ci);
        right_image_pub_.publish(right_image, right_ci);

  }
  }

  void Camera1394Driver::checkCameraInfo(const sensor_msgs::ImagePtr &image,
                                     sensor_msgs::CameraInfoPtr &ci,
                                     bool &calibration_matches,
                                     std::string camera_name)
  {
    if (!dev_->checkCameraInfo(*image, *ci))
      {
        // image size does not match: publish a matching uncalibrated
        // CameraInfo instead
        if (calibration_matches)
          {
            // warn user once
            calibration_matches = false;
            ROS_WARN_STREAM("[" << camera_name
                            << "] calibration does not match video mode "
                            << "(publishing uncalibrated data)");
          }
        ci.reset(new sensor_msgs::CameraInfo());
        ci->height = image->height;
        ci->width = image->width;
      }
    else if (!calibration_matches)
      {
        // calibration OK now
        calibration_matches = true;
        ROS_WARN_STREAM("[" << camera_name
                        << "] calibration matches video mode now");
      }
  }

  /** Read camera data.
   *
   * @param image points to camera Image message
   * @return true if successful, with image filled in
   */
  bool Camera1394Driver::read(sensor_msgs::ImagePtr &right_image,
                              sensor_msgs::ImagePtr &left_image)
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");
        dev_->readData(*left_image, *right_image);
        ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
      }
    catch (camera1394::Exception& e)
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void Camera1394Driver::reconfig(Config &newconfig, uint32_t level)
  {
    // Do not run concurrently with poll().  Tell it to stop running,
    // and wait on the lock until it does.
    reconfiguring_ = true;
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
      newconfig.frame_id = "camera";
    std::string tf_prefix = tf::getPrefixParam(priv_nh_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

    if (newconfig.multicam != "mono" && newconfig.multicam != "stereo_interlaced")
    {
      ROS_WARN("Invalid value for parameter multicam.  Defaulting to 'mono'.");
      newconfig.multicam = "mono";
    }

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
      {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
      }

    if (newconfig.multicam == "mono")
      {
      if (config_.multicam != newconfig.multicam)
          {
        //switching from stereo
            left_cinfo_.reset(new camera_info_manager::CameraInfoManager(camera_nh_, camera_name_, newconfig.camera_info_url));
            right_cinfo_.reset();

            left_image_pub_ = image_transport::CameraPublisher();
            right_image_pub_ = image_transport::CameraPublisher();

            left_it_.reset(new image_transport::ImageTransport(camera_nh_));
            right_it_.reset();

            left_image_pub_ = left_it_->advertiseCamera("image_raw", 1);

          }

      if (config_.camera_info_url != newconfig.camera_info_url)
      {
      // set the new URL and load CameraInfo (if any) from it
      if (left_cinfo_->validateURL(newconfig.camera_info_url))
        {
        left_cinfo_->loadCameraInfo(newconfig.camera_info_url);
        }
      else
        {
        // new URL not valid, use the old one
        newconfig.camera_info_url = config_.camera_info_url;
        }
      }
      }
    else
    {
      if (config_.multicam != newconfig.multicam)
      {
            // switching from mono
            left_cinfo_.reset(new camera_info_manager::CameraInfoManager(left_camera_nh_, camera_name_, newconfig.left_camera_info_url));
            right_cinfo_.reset(new camera_info_manager::CameraInfoManager(right_camera_nh_, camera_name_, newconfig.right_camera_info_url));

            left_image_pub_ = image_transport::CameraPublisher();
            right_image_pub_ = image_transport::CameraPublisher();

            left_it_.reset(new image_transport::ImageTransport(left_camera_nh_));
            right_it_.reset(new image_transport::ImageTransport(right_camera_nh_));

            left_image_pub_ = left_it_->advertiseCamera("image_raw", 1);
            right_image_pub_ = right_it_->advertiseCamera("image_raw", 1);
      }
      if (config_.left_camera_info_url != newconfig.left_camera_info_url)
      {
            // set the new URL and load CameraInfo (if any) from it
            if (left_cinfo_->validateURL(newconfig.left_camera_info_url))
              {
                left_cinfo_->loadCameraInfo(newconfig.left_camera_info_url);
              }
            else
              {
                // new URL not valid, use the old one
                newconfig.left_camera_info_url = config_.left_camera_info_url;
              }

            if (right_cinfo_->validateURL(newconfig.right_camera_info_url))
              {
                right_cinfo_->loadCameraInfo(newconfig.right_camera_info_url);
              }
            else
              {
                // new URL not valid, use the old one
                newconfig.right_camera_info_url = config_.right_camera_info_url;
              }

      }
    }

    if (state_ == Driver::CLOSED)
    {
      // open with new values
      if (openCamera(newconfig))
      {
        //update GUID string parameter
        newconfig.guid = camera_name_;
      }
    }

    if (state_ != Driver::CLOSED)       // openCamera() succeeded?
      {
        // configure IIDC features
        if (level & Levels::RECONFIGURE_CLOSE)
          {
            // initialize all features for newly opened device
            if (false == dev_->features_->initialize(&newconfig))
              {
                ROS_ERROR_STREAM("[" << camera_name_
                                 << "] feature initialization failure");
                closeCamera();          // can't continue
              }
          }
        else
          {
            // update any features that changed
            // TODO replace this with a dev_->reconfigure(&newconfig);
            dev_->features_->reconfigure(&newconfig);
          }
      }

    config_ = newconfig;                // save new parameters
    reconfiguring_ = false;             // let poll() run again

    if (config_.multicam == "mono")
      {
        ROS_DEBUG_STREAM("[" << camera_name_
                         << "] reconfigured: frame_id " << newconfig.frame_id
                         << ", camera_info_url " << newconfig.camera_info_url);
      }
    else
      {
        ROS_DEBUG_STREAM("[" << camera_name_
                         << "] reconfigured: frame_id " << newconfig.frame_id
                         << ", left_camera_info_url " << newconfig.left_camera_info_url
                         << ", right_camera_info_url " << newconfig.right_camera_info_url);
      }

  }


  /** driver initialization
   *
   *  Define dynamic reconfigure callback, which gets called
   *  immediately with level 0xffffffff.  The reconfig() method will
   *  set initial parameter values, then open the device if it can.
   */
  void Camera1394Driver::setup(void)
  {
    srv_.setCallback(boost::bind(&Camera1394Driver::reconfig, this, _1, _2));
  }


  /** driver termination */
  void Camera1394Driver::shutdown(void)
  {
    closeCamera();
  }

}; // end namespace camera1394_driver
