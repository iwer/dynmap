/**
 * @file RGBDemoCalibration.cpp
 *
 *  @date 11.11.2012
 *      @author Iwer Petersen
 */

#include "calibration/RGBDemoCalibration.h"
#include <ntk/projector/calibration.h>
#include <ntk/camera/calibration.h>
#include <ntk/utils/opencv_utils.h>

RGBDemoCalibration::RGBDemoCalibration(const char * cameraCalibrationFile,
        const char * projectorCalibrationFile) :
        cameraCalibrationFile_(cameraCalibrationFile), projectorCalibrationFile_(
                projectorCalibrationFile) {
    loadCalibration();
}

RGBDemoCalibration::~RGBDemoCalibration() {
    // TODO Auto-generated destructor stub
}

void toEigen(cv::Mat1d &mat, Eigen::Matrix3f &ep) {

      for (int r = 0; r < mat.rows; ++r)
        for (int c = 0; c < mat.cols; ++c)
          ep(r,c) = mat(r,c);

}

void RGBDemoCalibration::loadProjectorCalibration() {
    Eigen::Matrix3f projectorR_, projectorT_;
//    Eigen::Matrix3f rotz180;
//    rotz180 << -1, 0, 0, 0, -1, 0, 0, 0, 1;

    ntk::RGBDCalibration camera_calib;
    ntk::ProjectorCalibration projector_calib;

    camera_calib.loadFromFile(cameraCalibrationFile_);
    projector_calib.loadFromFile(projectorCalibrationFile_);

    projectorSize_ = projector_calib.proj_size;

    toEigen(projector_calib.intrinsics, projectorIntrinsics_);
//    std::cout << projectorIntrinsics_ << std::endl;

    toEigen(projector_calib.R, projectorR_);
    toEigen(projector_calib.T, projectorT_);
    //    projectorR_ = projectorR_ * rotz180;

    // There is a more elegant solution for this...
    projectorExtrinsics_(0, 0) = projectorR_(0, 0);
    projectorExtrinsics_(0, 1) = projectorR_(0, 1);
    projectorExtrinsics_(0, 2) = projectorR_(0, 2);
    projectorExtrinsics_(0, 3) = projectorT_(0, 0);
    projectorExtrinsics_(1, 0) = projectorR_(1, 0);
    projectorExtrinsics_(1, 1) = projectorR_(1, 1);
    projectorExtrinsics_(1, 2) = projectorR_(1, 2);
    projectorExtrinsics_(1, 3) = projectorT_(1, 0);
    projectorExtrinsics_(2, 0) = projectorR_(2, 0);
    projectorExtrinsics_(2, 1) = projectorR_(2, 1);
    projectorExtrinsics_(2, 2) = projectorR_(2, 2);
    projectorExtrinsics_(2, 3) = projectorT_(2, 0);
    projectorExtrinsics_(3, 0) = 0;
    projectorExtrinsics_(3, 1) = 0;
    projectorExtrinsics_(3, 2) = 0;
    projectorExtrinsics_(3, 3) = 1;
}

void RGBDemoCalibration::loadCameraCalibration() {
    cameraIntrinsics_.setIdentity();
    cameraExtrinsics_.setIdentity();
}

void RGBDemoCalibration::loadCalibration() {
    loadProjectorCalibration();
    loadCameraCalibration();
}

Eigen::Matrix4f RGBDemoCalibration::getCameraExtrinsics() const {
    return (cameraExtrinsics_);
}

Eigen::Matrix3f RGBDemoCalibration::getCameraIntrinsics() const {
    return (cameraIntrinsics_);
}

Eigen::Matrix4f RGBDemoCalibration::getProjectorExtrinsics() const {
    return (projectorExtrinsics_);
}

Eigen::Matrix3f RGBDemoCalibration::getProjectorIntrinsics() const {
    return (projectorIntrinsics_);
}

cv::Size RGBDemoCalibration::getProjectorSize() const {
    return (projectorSize_);
}


