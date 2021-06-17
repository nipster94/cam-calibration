#ifndef STEREO_CALIBRATION_H
#define STEREO_CALIBRATION_H


#include "ros/ros.h"
#include "ros/package.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/aruco.hpp>
//#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

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

enum image_side{
    LEFT,
    RIGHT
};

struct camera_details{
    image_side image_side_;
    cv::Mat k_Matrix_;
    cv::Mat distanceCoefficients;
    cv::Mat homography_;
    cv::Mat camera_matrix_;
    cv::Mat rectification_matrix_;
    cv::Mat map_1_;
    cv::Mat map_2_;
};

class StereoCalibration
{
public:
    StereoCalibration();
    void execute(void);

private:
    std::string pattern_l_;
    std::string pattern_r_;
    std::string directory_path_;

    std::string save_image_path_;
    bool save_images_ = true;
    bool useCalibrated = false;
    int good_image_count_ = 0;
    int number_of_images_;

    std::vector<std::string> left_img_paths_;
    std::vector<std::string> right_img_paths_;

    std::vector<std::vector<cv::Point2f> > allFoundLeftConners;   // this is 2D in left images
    std::vector<std::vector<cv::Point2f> > allFoundRightConners;  // this is 2D in right images
    std::vector<std::vector<cv::Point3f> > worldConerCoordinates;
    std::vector<cv::Point3f> coners3D;                            // 3D points of the board
    std::vector<cv::Mat> savedLeftImages;                         // if the conners are detected we keep it save later
    std::vector<cv::Mat> savedRightImages;                        // if the conners are detected we keep it save later

    float calibSquareDimentions;                                  // in meters
    cv::Size chessBoardDimensions;

    std::vector<std::string> getImagePaths(std::string directory_path_, std::string pattern_);

    bool getChessboardCorners(cv::Mat image_,  image_side image_detail);
    void createKnownBoardPositions();

    double checkCalibrationQuality(camera_details left_camera_, camera_details right_camera_,
                                   cv::Mat fundermental_matrix);

    void computeRectification(camera_details &left_camera_, camera_details &right_camera_,
                              cv::Mat rotational, cv::Mat traslational, cv::Mat fundermental, bool useCalibrated);

    void drawRectifiedImages(camera_details left_camera_, camera_details right_camera_,
                             bool isVerticalStereo, bool useCalibrated);

    void drawCorners();

};

#endif
