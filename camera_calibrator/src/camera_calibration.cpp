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
#include "camera_calibrator.h"
#include <iomanip>
#include <sstream>
CameraCalibrator::CameraCalibrator(int squaresX, int squaresY, float squareLength, float markerLength, const char *camera_name, const char *output_path)
    : squaresX_(squaresX),
      squaresY_(squaresY),
      squareLength_(squareLength),
      markerLength_(markerLength),
      camera_name_(camera_name),
      outputFile_(output_path)
{
    detectorParams_ = cv::makePtr<cv::aruco::DetectorParameters>();
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(6));
    charucoboard_ = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, &dictionary_);
    board_ = charucoboard_.staticCast<cv::aruco::Board>();
    projectionMatrix_ = cv::Mat::eye(3, 4, CV_64F);
}

bool CameraCalibrator::cal_corners(const cv::Mat &img)
{

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::Mat currentCharucoCorners, currentCharucoIds, imageCopy;

    cv::aruco::detectMarkers(img, cv::makePtr<cv::aruco::Dictionary>(dictionary_), corners, ids, detectorParams_, rejected);

    if (ids.size() <= 0)
        return false;

    cv::aruco::interpolateCornersCharuco(corners, ids, img, charucoboard_, currentCharucoCorners,
                                         currentCharucoIds);
    std::cout << "Frame captured" << frame_count_ << "\n";
    frame_count_++;
    allCorners_.push_back(corners);
    allIds_.push_back(ids);
    allImgs_.push_back(img.clone());
    imgSize_ = img.size();

    if (draw_)
    {
        img.copyTo(imageCopy);
        cv::aruco::drawDetectedMarkers(imageCopy, corners);
        if (currentCharucoCorners.total() > 0)
            cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
    }

    if (allIds_.size() > 20)
    {
        return true;
    }

    return false;
}

void CameraCalibrator::cal_calibration()
{

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<std::vector<cv::Point2f>> allCornersConcatenated;
    std::vector<int> allIdsConcatenated;
    std::vector<int> markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners_.size());

    for (unsigned int i = 0; i < allCorners_.size(); i++)
    {
        markerCounterPerFrame.push_back((int)allCorners_[i].size());
        for (unsigned int j = 0; j < allCorners_[i].size(); j++)
        {
            allCornersConcatenated.push_back(allCorners_[i][j]);
            allIdsConcatenated.push_back(allIds_[i][j]);
        }
    }
    arucoRepErr_ = cv::aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                                   markerCounterPerFrame, board_, imgSize_, cameraMatrix_,
                                                   distCoeffs_, cv::noArray(), cv::noArray(), calibrationFlags_);

    int nFrames = static_cast<int>(allCorners_.size());
    std::vector<cv::Mat> allCharucoCorners;
    std::vector<cv::Mat> allCharucoIds;
    std::vector<cv::Mat> filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);
    for (int i = 0; i < nFrames; i++)
    {
        // interpolate using camera parameters
        cv::Mat currentCharucoCorners, currentCharucoIds;
        cv::aruco::interpolateCornersCharuco(allCorners_[i], allIds_[i], allImgs_[i], charucoboard_,
                                             currentCharucoCorners, currentCharucoIds, cameraMatrix_,
                                             distCoeffs_);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs_[i]);
    }

    if (allCharucoCorners.size() < 4)
    {
        throw std::runtime_error("Not enough corners for calibration, please restart");
        return;
    }
    repError_ =
        cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard_, imgSize_,
                                          cameraMatrix_, distCoeffs_, rvecs, tvecs, calibrationFlags_);

    cameraMatrix_(cv::Rect(0, 0, 3, 3)).copyTo(projectionMatrix_(cv::Rect(0, 0, 3, 3)));
    tvecs[0].copyTo(projectionMatrix_(cv::Rect(3, 0, 1, 3)));
}

bool CameraCalibrator::save_calibration_file()
{
    cv::FileStorage fs(outputFile_, cv::FileStorage::WRITE);
    if (!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);
    fs << "calibration_time" << buf;
    fs << "image_width" << imgSize_.width;
    fs << "image_height" << imgSize_.height;
    fs << "camera_name" << camera_name_;
    fs << "distortion_model" << "plumb_bob";
    fs << "camera_matrix" << cameraMatrix_;
    fs << "distortion_coefficients" << distCoeffs_;
    fs << "rectification_matrix" << cv::Mat::eye(3, 3, CV_64F);
    fs << "projection_matrix" << projectionMatrix_;
    fs << "avg_reprojection_error" << repError_;
    fs << "aruco_reprojection_error" << arucoRepErr_;
    return true;
}
