/*************************************************************************************************************************
 * Copyright 2024 Grifcc&Kylin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/
#include "image_subscriber.h"
#include "camera_calibrator.h"

ImageSubscriber::ImageSubscriber(ros::NodeHandle &nh)
    : nh_(nh), it_(nh)
{
    // 订阅 image 话题
    image_sub_ = it_.subscribe("/mipi_cam/image_raw", 1, &ImageSubscriber::imageCallback, this);

    // 订阅 camera_info 话题
    camera_info_sub_ = nh_.subscribe("/mipi_cam/camera_info", 1, &ImageSubscriber::cameraInfoCallback, this);

    camera_calibrator_ = new CameraCalibrator(5, 7, 0.032, 0.020,"mipi_cam", "/root/kylin/camera_calibration/camera_calibrator/config/calibration.yaml");
    calibration_status_ = false;
}

void ImageSubscriber::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

    current_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (!calibration_status_)
    {
        calibration_status_ = camera_calibrator_->cal_corners(current_image_);
        if (calibration_status_)
        {
            image_sub_.shutdown();
            ROS_INFO_STREAM("image enough to calibrate,Start calculate calibration");
            camera_calibrator_->cal_calibration();
            ROS_INFO_STREAM("Calibrate Done,Start write calibration");
            camera_calibrator_->save_calibration_file();
            ROS_INFO_STREAM("write calibration done");
        }
    }
}

void ImageSubscriber::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    current_camera_info_ = *msg;
    // camera_info_received_ = true;
}
