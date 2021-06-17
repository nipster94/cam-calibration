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
#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <numeric>
#include <time.h>
#include <ctime>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <limits>
#include <numeric>
#include <time.h>
#include <ctime>

using ns = std::chrono::nanoseconds;
using get_time = std::chrono::steady_clock ;

const float calibSquareDimentions = 0.102f; // in meters
const cv::Size chessBoardDimensions = cv::Size(8,6);

image_transport::Publisher publisher_;
image_transport::Publisher left_image_pub_;
image_transport::Publisher right_image_pub_;


template<class bidiiter>
bidiiter random_unique(bidiiter begin, bidiiter end, size_t num_random) {
    size_t left = std::distance(begin, end);
    while (num_random--) {
        bidiiter r = begin;
        std::advance(r, rand()%left);
        std::swap(*begin, *r);
        ++begin;
        --left;
    }
    return begin;
}


void createArucoMarkers(){
    cv::Mat outputMat;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

}

void createKnownBoardPositions(cv::Size boardSize, float squareLenght, std::vector<cv::Point3f>& coners){
    for(int i = 0; i < boardSize.height; i++){
        for (int j = 0; j < boardSize.width; j++){
            coners.push_back(cv::Point3f(j*squareLenght,
                                         i*squareLenght,
                                         0.0f));
        }
    }
}

void getChessboardConers(std::vector<cv::Mat> images_, std::vector<std::vector<cv::Point2f>>& allFoundConners,
                         std::vector<cv::Mat>& savedImages,
                         bool showResults = false){
    for(std::vector<cv::Mat>::iterator iter = images_.begin();
        iter != images_.end(); iter++){
        std::vector<cv::Point2f> pointBuffer;
        bool found = cv::findChessboardCorners(*iter, cv::Size(8,6),
                                                pointBuffer,
                                                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);


        if (found){
            allFoundConners.push_back(pointBuffer);
            savedImages.push_back(*iter);
        }

        if (showResults){
            cv::drawChessboardCorners(*iter,chessBoardDimensions,pointBuffer,found);
            cv::imshow("conners", *iter);
            cv::waitKey(100);
        }


    }

    cv::destroyWindow("conners");


}

double computeReprojectionErrors(const std::vector< std::vector< cv::Point3f > >& objectPoints,
                                 const std::vector< std::vector< cv::Point2f > >& imagePoints,
                                 const std::vector< cv::Mat >& rvecs, const std::vector< cv::Mat >& tvecs,
                                 const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs){
    std::vector<cv::Point2f> imagePoints2;
    unsigned long i, totalPoints = 0;
    double totalErr = 0, err;
    std::vector<float> perViewErrors;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < objectPoints.size(); ++i) {
      cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
                        cameraMatrix, distCoeffs, imagePoints2);
      err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);
      int n = objectPoints[i].size();
      perViewErrors[i] = (float) std::sqrt(err*err/n);
      totalErr += err*err;
      totalPoints += n;
    }
    return std::sqrt(totalErr/totalPoints);
}

void saveAllRTVectors(std::vector<cv::Mat> rVectors, std::vector<cv::Mat> tVectors,unsigned int range){
  std::string fname_ = "/home/master01/calib_data/camera/random_RT/RTVectors_";
  fname_.append(std::to_string(range));
  fname_.append(".txt");

  bool init = true;
  for (int index = 0; index < rVectors.size(); ++index) {

    cv::Mat rMat = rVectors[index];
    cv::Mat tMat = tVectors[index];

    if (init) {
      cv::FileStorage writeF(fname_, cv::FileStorage::WRITE);
      writeF << "index" << index;
      writeF << "R" << rMat;
      writeF << "t" << tMat;
      init = false;
    }
    else {
      cv::FileStorage appendF(fname_, cv::FileStorage::APPEND);
      appendF << "index" << index;
      appendF << "R" << rMat;
      appendF << "t" << tMat;
    }
  }

}



//std::vector<cv::Mat> calibrationImages,
void cameraCalibration(std::vector<std::vector<cv::Point2f>> checkerBoardImagePoints,
                       cv::Size boardSize,
                       float squareLength,
                       cv::Mat& cameraMatrix,
                       cv::Mat& distanceCoefficients, double& reprojectionError,unsigned int range){

//    std::vector<std::vector<cv::Point2f>> checkerBoardImagePoints;
//    getChessboardConers(calibrationImages,checkerBoardImagePoints,false);

    std::vector<std::vector<cv::Point3f>> worldConnerCoordinates(1);
    createKnownBoardPositions(boardSize,squareLength,worldConnerCoordinates[0]);

    worldConnerCoordinates.resize(checkerBoardImagePoints.size(),worldConnerCoordinates[0]);
    std::cout << "random point size is : " << checkerBoardImagePoints.size() << "\n";
    std::cout << "world coordinate size : " << worldConnerCoordinates.size() << std::endl;

    std::vector<cv::Mat> rVectors,tVectors;

    distanceCoefficients = cv::Mat::zeros(8,1,CV_64F);

    cv::calibrateCamera(worldConnerCoordinates,
                        checkerBoardImagePoints,
                        boardSize,
                        cameraMatrix, distanceCoefficients,
                        rVectors,tVectors);

    std::cout << "r vector size : " << rVectors.size()
              << " t vectors size : " << tVectors.size() << "\n";

    std::cout << "done calibration" << "\n";

    saveAllRTVectors(rVectors,tVectors,range);

    reprojectionError = computeReprojectionErrors(worldConnerCoordinates,checkerBoardImagePoints,
                                                  rVectors,tVectors,
                                                  cameraMatrix, distanceCoefficients);


    std::cout <<  "Reprojection error : " << reprojectionError << '\n';

}



bool saveCalibrations(std::string name, cv::Mat camerMatrix, cv::Mat distanceCoefficients){

    std::ofstream outStream(name);

    if(outStream){
        uint16_t rows = camerMatrix.rows;
        uint16_t cols =camerMatrix.cols;

        for(int r = 0; r < rows; r++){
            for(int c =0; c < cols; c++){
                double value = camerMatrix.at<double>(r,c);
                outStream << value << std::endl;
            }
        }

        rows = distanceCoefficients.rows;
        cols = distanceCoefficients.cols;

        for(int r = 0; r < rows; r++){
            for(int c = 0; c < cols; c++){
                double value = distanceCoefficients.at<double>(r,c);
                outStream << value << std::endl;
            }
        }

        outStream.close();
        return true;

    }

    return  false;
}

std::vector<cv::Mat> readImages(std::string file_path_){
    std::vector<cv::Mat> inputImages;
    std::vector<cv::String> image_file_path_list_;
    std::string pattern_ = file_path_ + "/left-*.png";

//    std::string pattern_ = file_path_ + "/right-*.png";

    cv::glob(pattern_,image_file_path_list_,false);
//for(size_t i = 0; i < image_file_path_list_.size(); i++)
    for(size_t i = 0; i < image_file_path_list_.size(); i++){
        std::cout << "file name : " << image_file_path_list_[i] << std::endl;

        cv::Mat input_ = cv::imread(image_file_path_list_[i]);
        cv::Mat output_1, output_;
        output_ = input_;
//        cv::resize(input_,output_1,cv::Size(input_.cols*0.5,input_.rows*0.5),CV_INTER_AREA);
//        cv::flip(output_1,output_,-1);
//        std::cout << "height : " << input_.rows << " width : " << input_.cols << std::endl;
//        std::cout << "height : " << output_.rows << " width : " << output_.cols << std::endl;

        inputImages.push_back(output_);

    }

    std::cout << "size : " << inputImages.size() << std::endl;

    return inputImages;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_calibration");

    // initialize this node
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    std::string file_name_ = "/home/master01/rosbags/calibrationdata";
    std::vector<cv::Mat> images_ = readImages(file_name_);

    bool tmpPublisher = false;
    ros::Rate pub_rate(2);
    if (tmpPublisher){
        left_image_pub_ = it.advertise("stereo/left/image_raw",1);
        right_image_pub_ = it.advertise("stereo/right/image_raw",1);

        for(unsigned int i = 0; i < images_.size(); i++){
            while (nh.ok()) {
                std_msgs::Header  header;
                header.stamp = ros::Time::now();

                cv::Mat left_img = images_[i];
                sensor_msgs::ImagePtr left_msg_ = cv_bridge::CvImage(header, "bgr8", left_img).toImageMsg();
//                sensor_msgs::ImagePtr  right_msg_ = cv_bridge::CvImage(header, "bgr8", right_img).toImageMsg();

                left_image_pub_.publish(left_msg_);

                ros::spinOnce();
                pub_rate.sleep();
                break;

            }
        }

    }
    else {
//        cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
//        cv::Mat distanceCoefficients;

        std::vector<std::string> debug_str_values_;

        std::vector<std::vector<cv::Point2f>> markerConners,rejectedCandidates;

    //    cv::namedWindow("Images",CV_WINDOW_AUTOSIZE);
//        std::vector<cv::Mat> rVectors,tVectors;
        std::vector<std::vector<cv::Point2f>> foundPoints;
        std::vector<cv::Mat> savedImages_;
        getChessboardConers(images_,foundPoints, savedImages_, true);

        std::cout << "found point size : " << foundPoints.size() << std::endl;
        std::cout << "number of good images : " << savedImages_.size() << std::endl;

        bool exit = false;
        int range = 10;
        while (!exit) {
          auto start = get_time::now();

          if(foundPoints.size() - range <= 10){
            range = foundPoints.size();
            exit = true;
          }

          std::cout << "Current image point size is : " << range << "\n";
          double reprojectionError = 0;
          cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
          cv::Mat distanceCoefficients;

          std::vector<std::vector<cv::Point2f>> shufflePoint = foundPoints;
          random_unique(shufflePoint.begin(),shufflePoint.end(),range);
          std::vector<std::vector<cv::Point2f>> randomPoints;

          for(int i=0; i<range; ++i) {
              std::vector<cv::Point2f> pointSet = shufflePoint[i];
              randomPoints.push_back(pointSet);
          }

          std::cout << "random point size is : " << randomPoints.size() << "\n";
          cameraCalibration(randomPoints,
                            chessBoardDimensions,
                            calibSquareDimentions,
                            cameraMatrix,
                            distanceCoefficients,reprojectionError,range);

          std::cout << cameraMatrix << std::endl;
          std::cout << distanceCoefficients << std::endl;

          auto end = get_time::now();
          auto diff = end - start;
          double elapsedTime = std::chrono::duration_cast<ns>(diff).count()/1000000;
          std::cout << "Elapsed time is :  "<< elapsedTime  <<" ms "<< std::endl;
          std::cout << "=========================================" << std::endl;

          std::string fname_ = "/home/master01/calib_data/camera/random_camera/calib_960_600_102_";
          fname_.append(std::to_string(range));
          fname_.append(".txt");

  //        saveCalibrations(fname_,cameraMatrix,distanceCoefficients);

//          std::cout << "saving stuff" << std::endl;

          std::string debug = std::to_string(range) + " "
                            + std::to_string(reprojectionError) + " "
                            + std::to_string(elapsedTime);
          debug_str_values_.push_back(debug);

          cv::FileStorage fs(fname_, cv::FileStorage::WRITE);
          fs << "K" << cameraMatrix;
          fs << "D" << distanceCoefficients;
          fs << "board_width" << chessBoardDimensions.width;
          fs << "board_height" << chessBoardDimensions.height;
          fs << "square_size" << calibSquareDimentions;
          fs << "reprojection error" << reprojectionError;
          fs << "number of points considered" << range;
          fs << "Elapsed time in seconds"<< elapsedTime/1000 ;

          range += 10;

        }

        std::string debug_fname = "/home/master01/calib_data/camera/debug_file.txt";
        std::ofstream outStream(debug_fname);

        if(outStream){
            for(int r = 0; r < debug_str_values_.size(); r++){
                std::string value = debug_str_values_[r];
                outStream << value << std::endl;
            }
            outStream.close();
        }


    }



    return 0;
}




