
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include "sp_codec.h"
#include "sp_vio.h"
#include "sp_sys.h"
#include "sp_display.h"


using namespace std;
using namespace cv;

bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

bool saveCameraParams(const std::string &filename, cv::Size imageSize, float aspectRatio, int flags,
                                    const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, double totalAvgErr) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if (flags & cv::CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if (flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }
    fs << "flags" << flags;
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "avg_reprojection_error" << totalAvgErr;
    return true;
}


int main(int argc, char *argv[]) {

    int squaresX = 5;
    int squaresY = 7;
    float squareLength = 0.032;
    float markerLength = 0.020;
    string outputFile = "/root/kylin/camera_calibration/calibration.yaml";

    int calibrationFlags = 0;
    calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
    float aspectRatio = 1;
    // if(parser.has("a")) {
    //     calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
    //     aspectRatio = parser.get<float>("a");
    // }
    // if(parser.get<bool>("zt")) calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
    // if(parser.get<bool>("pc")) calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;

    Ptr<aruco::DetectorParameters> detectorParams = makePtr<aruco::DetectorParameters>();
    bool refindStrategy = true;
    int imW = 640;
    int imH = 480;
    int ret = 0;
    sp_sensors_parameters parms;
    parms.fps = -1;
    parms.raw_width = 1920;
    parms.raw_height = 1080;
    // init module
    void *vio_object = sp_init_vio_module();
    ret = sp_open_camera_v2(vio_object, 0, -1, 1, &parms, &imW, &imH);
    if (ret != 0)
    {
        printf("[Error] sp_open_camera failed!\n");
        sp_vio_close(vio_object);
        sp_release_vio_module(vio_object);
    }
    printf("sp_open_camera success!\n");
    int waitTime = 10;
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(6));
    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, &dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();
    // collect data from each frame
    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector< Mat > allImgs;
    Size imgSize;

    char *image_buffer = (char *)malloc(FRAME_BUFFER_SIZE(imW, imH));
    Mat nv12_image(imH * 3 / 2, imW, CV_8UC1, image_buffer);
    Mat image(imH, imW, CV_8UC3);
    Mat imageCopy(imH, imW, CV_8UC3);
    int i = 0;

    while(1) {
        memset(image_buffer, 0, FRAME_BUFFER_SIZE(imW, imH));
        sp_vio_get_frame(vio_object,image_buffer,imW,imH,2000);
        cv::cvtColor(nv12_image, image, cv::COLOR_YUV2BGR_NV12);
        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        // detect markers 
        aruco::detectMarkers(image, makePtr<aruco::Dictionary>(dictionary), corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);
        // interpolate charuco corners
        Mat currentCharucoCorners, currentCharucoIds;
        if(ids.size() > 0)
            aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
                                             currentCharucoIds);
        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners);

        if(currentCharucoCorners.total() > 0)
            aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);

        putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        imwrite(cv::format("/root/kylin/camera_calibration/frame/result_%d.jpg", i), imageCopy);

        // char key = (char)waitKey(waitTime);
        // if(key == 27) break;
        // if(key == 'c' && ids.size() > 0) {
        //     std::cout << "Frame captured" << endl;
        //     allCorners.push_back(corners);
        //     allIds.push_back(ids);
        //     allImgs.push_back(image);
        //     imgSize = image.size();
        // }
         if(ids.size() > 0) {
            std::cout << "Frame captured" << endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(image);
            imgSize = image.size();
        }
        if(allIds.size()>20)
            break;
    }



    if(allIds.size() < 1) {
        cerr << "Not enough captures for calibration" << endl;
        return 0;
    }

    Mat cameraMatrix, distCoeffs;
    vector< Mat > rvecs, tvecs;
    double repError;

    if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = aspectRatio;
    }

    // prepare data for calibration
    vector< vector< Point2f > > allCornersConcatenated;
    vector< int > allIdsConcatenated;
    vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                              markerCounterPerFrame, board, imgSize, cameraMatrix,
                                              distCoeffs, noArray(), noArray(), calibrationFlags);

    // prepare data for charuco calibration
    int nFrames = (int)allCorners.size();
    vector< Mat > allCharucoCorners;
    vector< Mat > allCharucoIds;
    vector< Mat > filteredImages;
    allCharucoCorners.reserve(nFrames);
    allCharucoIds.reserve(nFrames);

    for(int i = 0; i < nFrames; i++) {
        // interpolate using camera parameters
        Mat currentCharucoCorners, currentCharucoIds;
        aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                         distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
        filteredImages.push_back(allImgs[i]);
    }

    if(allCharucoCorners.size() < 4) {
        cerr << "Not enough corners for calibration" << endl;
        return 0;
    }

    // calibrate camera using charuco
    repError =
        aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
                                      cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

    bool saveOk =  saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
                                    cameraMatrix, distCoeffs, repError);
    if(!saveOk) {
        cerr << "Cannot save output file" << endl;
        return 0;
    }

    cout << "Rep Error: " << repError << endl;
    cout << "Rep Error Aruco: " << arucoRepErr << endl;
    cout << "Calibration saved to " << outputFile << endl;


    free(image_buffer);
    sp_vio_close(vio_object);
    sp_release_vio_module(vio_object);
    return 0;
}
