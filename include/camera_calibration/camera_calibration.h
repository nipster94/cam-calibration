#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include "ros/ros.h"
#include "ros/package.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
//#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/image_encodings.h>        //Converting ROS to OpenCV images
#include <cv_bridge/cv_bridge.h>                //Converting ROS to OpenCV images
#include <image_transport/image_transport.h>    //Publishing and subscribing to images in ROS

#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <limits>
#include <numeric>
#include <time.h>
#include <ctime>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <jsoncpp/json/json.h>

#include <boost/thread.hpp>

class CameraCalibration
{
public:
    CameraCalibration();

    void execute(void);
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    cv_bridge::CvImagePtr cv_ptr;
    image_transport::Subscriber left_sub_, right_sub_;

    std::vector<std::vector<cv::Point2f> > allFoundConners;  // this is 2D in images...
    std::vector<std::vector<cv::Point3f> > worldConnerCoordinates;
    std::vector<cv::Point3f> conners3D;
    std::vector<cv::Mat> savedImages;                       // if the conners are detected we keep it save later

    float calibSquareDimentions;  // in meters
    cv::Size chessBoardDimensions;
    int perviousConnerSize_ = 0;

    boost::thread* cameraCalibrationThread;

    std::vector<cv::Mat> rVectors,tVectors;
    cv::Mat cameraMatrix;
    cv::Mat distanceCoefficients;

    bool stopThread;
    bool calibrationInProgress;

    boost::mutex calibrationMutex;
    boost::mutex mainThreadMutex;
    double reprojectionError_;

    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg);

    void getChessboardConers(cv::Mat image_, bool showResults = false);
    bool haveEnoughImages();

    std::vector<cv::Point3f> createKnownBoardPositions();

    void performCameraCalibration();

    double computeReprojectionErrors(const std::vector< std::vector< cv::Point3f > >& objectPoints,
                                     const std::vector< std::vector< cv::Point2f > >& imagePoints,
                                     const std::vector< cv::Mat >& rvecs, const std::vector< cv::Mat >& tvecs,
                                     const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs);

};



#endif
