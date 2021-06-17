#include "camera_calibration.h"

CameraCalibration::CameraCalibration():
    it_(nh_)
{
    left_sub_ = it_.subscribe("/stereo/left/image_raw", 1, &CameraCalibration::leftImageCallback,this);
    calibSquareDimentions = 0.102f;
    chessBoardDimensions = cv::Size(8,6);
    stopThread = false;
    cameraMatrix = cv::Mat::eye(3,3,CV_64F);
    distanceCoefficients = cv::Mat::zeros(8,1,CV_64F);
    calibrationInProgress = false;
    reprojectionError_ = 0.0;
}

void CameraCalibration::execute(){
    ROS_INFO("Starting calibration in new thread");
    conners3D = createKnownBoardPositions();
    cameraCalibrationThread = new boost::thread(boost::bind(&CameraCalibration::performCameraCalibration,
                                                            this));
//    cameraCalibrationThread->join();

    ros::Rate rate(20);
    ROS_INFO("Starting the main thread");
    while (ros::ok()) {

        if (calibrationInProgress) {
            ROS_INFO("CALIBRATION IN PROGRESS");
        }

        if(haveEnoughImages() && !calibrationInProgress){
            // need to lock all these values
            const std::vector<std::vector<cv::Point3f> > objectPoints = worldConnerCoordinates;
            const std::vector<std::vector<cv::Point2f> > imagePoints = allFoundConners;
            const std::vector<cv::Mat> rvecs = rVectors;
            const std::vector<cv::Mat> tvecs = tVectors;
            const cv::Mat cameraMat = cameraMatrix;
            const cv::Mat distCoeff = distanceCoefficients;

            double error = computeReprojectionErrors(objectPoints,imagePoints,rvecs,tvecs,cameraMat,distCoeff);

            std::cout << "Calibration error: " << error << "\n";

        }
        rate.sleep();
        ros::spinOnce();
    }

    stopThread = true;
    cameraCalibrationThread->detach();


    std::string out_file = "/home/master01/test_calibration.txt";
    std::cout << "saving stuff" << std::endl;
    cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
    fs << "K" << cameraMatrix;
    fs << "D" << distanceCoefficients;
    fs << "board_width" << chessBoardDimensions.width;
    fs << "board_height" << chessBoardDimensions.height;
    fs << "square_size" << calibSquareDimentions;
    fs << "reprojection error" << reprojectionError_;
    printf("Done Calibration\n");

}

void CameraCalibration::leftImageCallback(const sensor_msgs::ImageConstPtr &msg){
//    auto start = get_time::now();

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat originalLeftImage = cv_ptr->image;
    getChessboardConers(originalLeftImage,true);


}

void CameraCalibration::getChessboardConers(cv::Mat image_, bool showResults){
    std::vector<cv::Point2f> pointBuffer;
    bool found = cv::findChessboardCorners(image_, cv::Size(8,6),
                                           pointBuffer,
                                           cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);


    if (found){
        allFoundConners.push_back(pointBuffer);
        savedImages.push_back(image_);
    }

    if (showResults){
        cv::drawChessboardCorners(image_, chessBoardDimensions, pointBuffer, found);
        cv::imshow("coners", image_);
        cv::waitKey(10);
    }
}

bool CameraCalibration::haveEnoughImages(){
    return ((allFoundConners.size() > 15) &&
           (allFoundConners.size() == savedImages.size()) &&
           (allFoundConners.size() % 15 == 0));
}

std::vector<cv::Point3f> CameraCalibration::createKnownBoardPositions(){
    std::vector<cv::Point3f> conners3D;
    for(int i = 0; i < chessBoardDimensions.height; i++){
        for (int j = 0; j < chessBoardDimensions.width; j++){
            conners3D.push_back(cv::Point3f(j*calibSquareDimentions,
                                            i*calibSquareDimentions,
                                            0.0f));
        }
    }

    return conners3D;
}


void CameraCalibration::performCameraCalibration(){
    unsigned long previousSize = 0;
    while (!stopThread) {
        calibrationMutex.lock();
        unsigned long currentSize = allFoundConners.size();
        worldConnerCoordinates.resize(currentSize,conners3D);
        const std::vector<std::vector<cv::Point3f>> objectPoints = worldConnerCoordinates;
        const std::vector<std::vector<cv::Point2f>> imagePoints = allFoundConners;

        if(currentSize > 10 &&
          (currentSize - previousSize) > 5){
            calibrationInProgress = true;

            std::cout << "world coordinate size : " << objectPoints.size() << std::endl;
            std::cout << "2D coordinate size : " << imagePoints .size() << std::endl;

            //reset matrices
            cameraMatrix = cv::Mat::eye(3,3,CV_64F);
            distanceCoefficients = cv::Mat::zeros(8,1,CV_64F);

            //do calibration
            cv::calibrateCamera(objectPoints,
                                imagePoints,
                                chessBoardDimensions,
                                cameraMatrix, distanceCoefficients,
                                rVectors,tVectors);
            previousSize = currentSize;
        }

        reprojectionError_ = computeReprojectionErrors(objectPoints,imagePoints,
                                                       rVectors,tVectors,
                                                       cameraMatrix,distanceCoefficients);
        std::cout << "Reprojection error from calib thread: " << reprojectionError_ << "\n";

        calibrationInProgress = false;
        calibrationMutex.unlock();
        //sleep for 5 seconds
        boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
    }

}

double CameraCalibration::computeReprojectionErrors(const std::vector< std::vector< cv::Point3f > >& objectPoints,
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
