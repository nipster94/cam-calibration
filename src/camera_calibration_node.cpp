#include "camera_calibration.h"

int main(int argc, char** argv)
{
    ROS_INFO("Starting Camera Calibration Node");
    ros::init(argc, argv, "camera_calibrator");
    CameraCalibration cameraCalibration;
    cameraCalibration.execute();
    return 0;
}
