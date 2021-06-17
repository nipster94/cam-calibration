#include "stereo_calibration.h"

StereoCalibration::StereoCalibration()
{
    pattern_l_ = "left-*.png";
    pattern_r_ = "right-*.png";
//    directory_path_ = "/home/nipun/rosbags/new_stuff_2/new_images/";
    directory_path_ = "/home/nipun/backup/calibrationdata/new/";
    save_image_path_ = "/home/nipun/backup/merged_calib_data/";
    calibSquareDimentions = 0.102f;
    chessBoardDimensions = cv::Size(8,6);
    number_of_images_ = 51;
}

void StereoCalibration::execute(){
    left_img_paths_ = getImagePaths(directory_path_,pattern_l_);
    right_img_paths_ = getImagePaths(directory_path_,pattern_r_);

    createKnownBoardPositions();
// left_img_paths_.size()
    for(int i = 0; i < number_of_images_; ++i){
        std::cout << left_img_paths_[i] << '\n';
        std::cout << right_img_paths_[i] << '\n';
        std::cout << "=================" << '\n';

        cv::Mat left_image_ = cv::imread(left_img_paths_[i],CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat right_image_ = cv::imread(right_img_paths_[i],CV_LOAD_IMAGE_UNCHANGED);

        bool found_l = getChessboardCorners(left_image_,image_side::LEFT);
        bool found_r = getChessboardCorners(right_image_,image_side::RIGHT);

        if(found_l && found_r){
            savedLeftImages.push_back(left_image_);
            savedRightImages.push_back(right_image_);
            good_image_count_ += 1;
            drawCorners();
        }
        else if (found_l && !found_r) {
            allFoundLeftConners.pop_back();
        }
        else if (!found_l && found_r) {
            allFoundRightConners.pop_back();
        }

    }

    cv::Mat M1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat M2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D1, D2, R, T, E, F;

    unsigned long currentSize = allFoundLeftConners.size();
    worldConerCoordinates.resize(currentSize,coners3D);
    std::cout << "\nRunning stereo calibration ...\n";
    // cv::stereoCalibrate(
    //    worldConerCoordinates, allFoundLeftConners, allFoundRightConners, M1, D1, M2, D2, savedLeftImages.back().size(), R, T, E, F,
    //     cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH,
    //     cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,1e-5));
    
    cv::stereoCalibrate(
       worldConerCoordinates, allFoundLeftConners, allFoundRightConners, M1, D1, M2, D2, 
       savedLeftImages.back().size(), R, T, E, F,
       cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH,
       cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 100,1e-5));


    std::string out_file = save_image_path_ + "calibration.txt";
    std::cout << "saving stuff" << std::endl;
    cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
    fs << "K1" << M1;
    fs << "D1" << D1;
    fs << "K2" << M2;
    fs << "D2" << D2;
    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
    fs << "board_width" << chessBoardDimensions.width;
    fs << "board_height" << chessBoardDimensions.height;
    fs << "square_size" << calibSquareDimentions;
//    fs << "reprojection error" << reprojectionError_;
    printf("Done Calibration\n");

    camera_details left_camera_;
    left_camera_.image_side_ = image_side::LEFT;
    left_camera_.k_Matrix_ = M1;
    left_camera_.distanceCoefficients = D1;

    camera_details right_camera_;
    right_camera_.image_side_ = image_side::RIGHT;
    right_camera_.k_Matrix_ = M2;
    right_camera_.distanceCoefficients = D1;

    double error = checkCalibrationQuality(left_camera_,right_camera_,F);
    std::cout << error << '\n';
    computeRectification(left_camera_,right_camera_,R,T,F,false);

    bool isVerticalStereo = !useCalibrated ? false :
                            fabs(right_camera_.camera_matrix_.at<double>(1,3)) > fabs(right_camera_.camera_matrix_.at<double>(0,3));

    drawRectifiedImages(left_camera_,right_camera_,isVerticalStereo,false);

    // drawRectifiedImages(left_camera_,right_camera_,false,false);



}

std::vector<std::string> StereoCalibration::getImagePaths(std::__cxx11::string directory_path_, std::__cxx11::string pattern_){

    std::vector<std::string> image_paths_;
    std::vector<cv::String> fn;
    pattern_ = directory_path_ + "/"+ pattern_;
    glob(pattern_, fn, false);
    size_t count = fn.size();
    for(size_t i=0; i<count; i++){
        image_paths_.push_back(fn[i]);
    }

    return image_paths_;
}

bool StereoCalibration::getChessboardCorners(cv::Mat image_, image_side image_detail){
    std::vector<cv::Point2f> pointBuffer;
    bool found = cv::findChessboardCorners(image_, cv::Size(8,6),
                                           pointBuffer,
                                           cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);


    if (found && image_detail ==  image_side::LEFT){
        allFoundLeftConners.push_back(pointBuffer);
    }
    else if (found && image_detail ==  image_side::RIGHT) {
        allFoundRightConners.push_back(pointBuffer);
    }

    return found;
}

void StereoCalibration::createKnownBoardPositions(){
    coners3D.clear();
    for(int i = 0; i < chessBoardDimensions.height; i++){
        for (int j = 0; j < chessBoardDimensions.width; j++){
            coners3D.push_back(cv::Point3f(j*calibSquareDimentions,
                                            i*calibSquareDimentions,
                                            0.0f));
        }
    }
}

void StereoCalibration::drawCorners(){
    // save_images_ = true;
    cv::Mat left_ = savedLeftImages.back();
    cv::Mat right_ = savedRightImages.back();

    if(left_.channels() < 3 && right_.channels() < 3){
        cv::cvtColor(left_,left_,cv::COLOR_GRAY2RGB);
        cv::cvtColor(right_,right_,cv::COLOR_GRAY2RGB);
    }

//    cv::cornerSubPix(left_, allFoundLeftConners.back(), cv::Size(5, 5), cv::Size(-1, -1),
//    cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
    cv::drawChessboardCorners(left_,chessBoardDimensions,allFoundLeftConners.back(),true);

//    cv::cornerSubPix(right_, allFoundRightConners.back(), cv::Size(5, 5), cv::Size(-1, -1),
//    cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
    cv::drawChessboardCorners(right_,chessBoardDimensions,allFoundRightConners.back(),true);

    cv::Mat mergeMat;
    cv::hconcat(left_,right_,mergeMat);

    if (save_images_){
        std::string path_ = save_image_path_ + std::to_string(good_image_count_) + ".png";
        std::cout << path_ << '\n';
        cv::imwrite(path_,mergeMat);
    }

    cv::imshow("All Coners",mergeMat);
    cv::waitKey(10);
}

double StereoCalibration::checkCalibrationQuality(camera_details left_camera_, camera_details right_camera_,
                                                  cv::Mat fundermental_matrix){
    std::vector<cv::Point3f> lines[2];
    double avgErr = 0;
    double N = chessBoardDimensions.width*chessBoardDimensions.height;
    int nframes = (int)worldConerCoordinates.size();

    for (int i = 0; i < nframes; i++) {
      std::vector<cv::Point2f> &pt0 = allFoundLeftConners[i];
      std::vector<cv::Point2f> &pt1 = allFoundRightConners[i];
      cv::undistortPoints(pt0, pt0, left_camera_.k_Matrix_, left_camera_.distanceCoefficients, cv::Mat(), left_camera_.k_Matrix_);
      cv::undistortPoints(pt1, pt1, right_camera_.k_Matrix_, right_camera_.distanceCoefficients, cv::Mat(), right_camera_.k_Matrix_);
      cv::computeCorrespondEpilines(pt0, 1, fundermental_matrix, lines[0]);
      cv::computeCorrespondEpilines(pt1, 2, fundermental_matrix, lines[1]);

      for (int j = 0; j < N; j++) {
        double err = fabs(pt0[j].x * lines[1][j].x + pt0[j].y * lines[1][j].y +
                          lines[1][j].z) +
                     fabs(pt1[j].x * lines[0][j].x + pt1[j].y * lines[0][j].y +
                          lines[0][j].z);
        avgErr += err;
      }

    }
    std::cout << "avg err = " << avgErr / (nframes * N) << std::endl;

    return avgErr / (nframes * N);
}

void StereoCalibration::computeRectification(camera_details &left_camera_, camera_details &right_camera_,
                                             cv::Mat rotational, cv::Mat traslational, cv::Mat fundermental, bool useCalibrated){
    cv::Mat R1, R2, P1, P2, map11, map12, map21, map22;
    cv::Rect roi1,roi2;
    if (useCalibrated){
        std::cout << "use calibration" << '\n';
        // cv::stereoRectify(left_camera_.k_Matrix_, left_camera_.distanceCoefficients,
        //                   right_camera_.k_Matrix_, right_camera_.distanceCoefficients,
        //                   savedLeftImages.back().size(), rotational, traslational,
        //                   R1, R2, P1, P2, cv::noArray(), 0);
        cv::stereoRectify(left_camera_.k_Matrix_, left_camera_.distanceCoefficients,
                    right_camera_.k_Matrix_, right_camera_.distanceCoefficients,
                    savedLeftImages.back().size(), rotational, traslational,
                    R1, R2, P1, P2, cv::noArray(), 
                    0,-1,cv::Size(960,600),&roi1,&roi2);
        // Precompute maps for cvRemap()
        cv::initUndistortRectifyMap(left_camera_.k_Matrix_, left_camera_.distanceCoefficients,
                                    R1, P1, savedLeftImages.back().size(), CV_16SC2, map11, map12);
        cv::initUndistortRectifyMap(right_camera_.k_Matrix_, right_camera_.distanceCoefficients,
                                    R2, P2, savedLeftImages.back().size(), CV_16SC2, map21, map22);
    }
    else {
        std::cout << "no calibration" << '\n';
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
        std::vector<cv::Point2f> allpoints[2];
        int nframes = (int)worldConerCoordinates.size();
        for (int i = 0; i < nframes; i++) {
          std::copy(allFoundLeftConners[i].begin(), allFoundLeftConners[i].end(),
               back_inserter(allpoints[0]));
          std::copy(allFoundRightConners[i].begin(), allFoundRightConners[i].end(),
               back_inserter(allpoints[1]));
        }
//        cv::Mat F = findFundamentalMat(allFoundLeftConners[0], allFoundRightConners[0], cv::FM_8POINT);
        cv::Mat F = findFundamentalMat(allpoints[0], allpoints[1], cv::FM_8POINT);

//        cv::Mat F = fundermental;
        std::cout << F << '\n';
        cv::Mat H1, H2;
//        cv::stereoRectifyUncalibrated(allFoundLeftConners[0], allFoundRightConners[0], F,
//                                      savedLeftImages.back().size(),
//                                      H1, H2, 3);

        cv::stereoRectifyUncalibrated(allpoints[0], allpoints[1], F,
                                      savedLeftImages.back().size(),
                                      H1, H2, 3);


        std::cout << H1 << '\n';
        std::cout << H2 << '\n';

        R1 = left_camera_.k_Matrix_.inv() * H1 * left_camera_.k_Matrix_;
        R2 = right_camera_.k_Matrix_.inv() * H2 *  right_camera_.k_Matrix_;

        std::cout << R1 << '\n';
        std::cout << R2 << '\n';

        // Precompute map for cvRemap()
        cv::initUndistortRectifyMap(left_camera_.k_Matrix_, left_camera_.distanceCoefficients,
                                    R1, P1, savedLeftImages.back().size(), CV_16SC2, map11, map12);
        cv::initUndistortRectifyMap(right_camera_.k_Matrix_, right_camera_.distanceCoefficients,
                                    R2, P2, savedLeftImages.back().size(), CV_16SC2, map21, map22);

        left_camera_.homography_ = H1;
        right_camera_.homography_ = H2;
    }

    left_camera_.camera_matrix_ = P1;
    left_camera_.rectification_matrix_ = R1;
    left_camera_.map_1_ = map11;
    left_camera_.map_2_ = map12;

    right_camera_.camera_matrix_ = P2;
    right_camera_.rectification_matrix_ = R2;
    right_camera_.map_1_ = map21;
    right_camera_.map_2_ = map22;

    std::cout << "======= end =======" << '\n';
}

void StereoCalibration::drawRectifiedImages(camera_details left_camera_, camera_details right_camera_,
                                            bool isVerticalStereo, bool useCalibrated){
    std::string line_images_dir_ = save_image_path_ + "lines/";
    std::string wrapped_images_dir_ = save_image_path_ + "wrapped/";
    int img_count_ = 1;
    cv::Mat pair_;
    if (!isVerticalStereo)
      pair_.create( savedLeftImages.back().size().height,
                    savedLeftImages.back().size().width * 2, CV_8UC3);
    else
      pair_.create(savedLeftImages.back().size().height * 2,
                   savedLeftImages.back().size().width, CV_8UC3);

    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
        -64, 128, 11, 100, 1000, 32, 0, 15, 1000, 16, cv::StereoSGBM::MODE_HH);

    for(int i = 0; i < number_of_images_; ++i){
        cv::Mat left_image_ = savedLeftImages[i];
        cv::Mat right_image_ = savedRightImages[i];
        cv::Mat left_rec_, right_rec_, disp_, vdisp_;

        cv::remap(left_image_, left_rec_, left_camera_.map_1_, left_camera_.map_2_, cv::INTER_LINEAR);
        cv::remap(right_image_, right_rec_, right_camera_.map_1_, right_camera_.map_2_, cv::INTER_LINEAR);

        if (!isVerticalStereo || useCalibrated){
            std::cout << "in the first if" << '\n';
            stereo->compute(left_rec_, right_rec_, disp_);
            cv::normalize(disp_, vdisp_, 0, 256, cv::NORM_MINMAX, CV_8U);
            cv::imshow("disparity", vdisp_);
        }

        if (!isVerticalStereo) {
            std::cout << "not isVerticalStereo" << '\n';
            cv::Mat part_ = pair_.colRange(0, left_image_.size().width);
            cv::cvtColor(left_rec_, part_, cv::COLOR_GRAY2BGR);
            part_ = pair_.colRange(left_image_.size().width, left_image_.size().width * 2);
            cv::cvtColor(right_rec_, part_, cv::COLOR_GRAY2BGR);
            for (int j = 0; j < left_image_.size().height; j += 16)
                cv::line(pair_, cv::Point(0, j),
                cv::Point(left_image_.size().width * 2, j),
                cv::Scalar(0, 255, 0));
        } else {
            std::cout << "isVerticalStereo" << '\n';
            cv::Mat part_ = pair_.rowRange(0, left_image_.size().height);
            cv::cvtColor(left_rec_, part_, cv::COLOR_GRAY2BGR);
            part_ = pair_.rowRange(left_image_.size().height, left_image_.size().height * 2);
            cv::cvtColor(right_rec_, part_, cv::COLOR_GRAY2BGR);
            for (int j = 0; j < left_image_.size().width; j += 16)
                cv::line(pair_, cv::Point(j, 0),
                cv::Point(j, left_image_.size().height * 2),
                cv::Scalar(0, 255, 0));
        }

//        cv::Mat tmp_ = left_image_.clone();
//        cv::cvtColor(tmp_,tmp_,cv::COLOR_GRAY2BGR);
//        cv::cvtColor(tmp_,tmp_,cv::COLOR_BGR2BGRA);

        cv::Mat rectified1(left_image_.size(), left_image_.type());
//        cv::Mat rectified1;
//        rectified1.create(tmp_.size(),CV_8UC4);
        cv::warpPerspective(left_image_, rectified1, left_camera_.homography_, left_image_.size(),
                            cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//        cv::imwrite("rectified1.jpg", rectified1);

        cv::Mat rectified2(right_image_.size(), right_image_.type());
        cv::warpPerspective(right_image_, rectified2, right_camera_.homography_, right_image_.size());
//        cv::imwrite("rectified2.jpg", rectified2);

        cv::Mat mergeMat;
        cv::hconcat(rectified1,rectified2,mergeMat);

        cv::imshow("rectified", pair_);
//         cv::imshow("rec1",rectified1);
//          cv::imshow("rec2",rectified2);
        cv::imshow("wraped",mergeMat);
        std::string lines_ = line_images_dir_ + "line_" + std::to_string(img_count_) + ".png";
        std::string wrapps_ = wrapped_images_dir_ + "wrap_" + std::to_string(img_count_) + ".png";
        std::string disp_str_ = line_images_dir_ + "disparity_" + std::to_string(img_count_) + ".png";
        std::string rect_left_ = line_images_dir_ + "rect_left_" + std::to_string(img_count_) + ".png";
        std::string rect_right_ = line_images_dir_ + "rect_right_" + std::to_string(img_count_) + ".png";

        // std::cout << lines_ << '\n';
        // std::cout << wrapps_ << '\n';
        // std::cout << disp_ << '\n';

        cv::imwrite(lines_, pair_);
        cv::imwrite(wrapps_, mergeMat);
        cv::imwrite(disp_str_, vdisp_);
        cv::imwrite(rect_left_, rectified1);
        cv::imwrite(rect_right_, rectified2);
        img_count_ += 1;

        if ((cv::waitKey() & 255) == 27) {
            break;
        }
    }
}


int main(int argc, char** argv)
{
    StereoCalibration stereo_calibration;
    stereo_calibration.execute();
    return 0;
}


