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
#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <iostream>

class CameraCalibrator
{

public:
    CameraCalibrator(int squaresX, int squaresY, float squareLength, float markerLength, const char *camera_name, const char *output_path);
    ~CameraCalibrator() {}

    bool cal_corners(const cv::Mat &img);
    void cal_calibration();
    bool save_calibration_file();

private:
    int squaresX_;
    int squaresY_;
    float squareLength_;
    float markerLength_;
    std::string camera_name_;
    std::string outputFile_;

    int calibrationFlags_ = 0;
    bool draw_ = false;
    int frame_count_ = 0;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
    cv::aruco::Dictionary dictionary_;
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard_;
    cv::Ptr<cv::aruco::Board> board_;
    std::vector<std::vector<std::vector<cv::Point2f>>> allCorners_;
    std::vector<std::vector<int>> allIds_;
    std::vector<cv::Mat> allImgs_;
    cv::Size imgSize_;

    cv::Mat cameraMatrix_, distCoeffs_, projectionMatrix_;
    double repError_;
    double arucoRepErr_;

};
