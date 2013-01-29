/**
 * @file RGBDemoCalibration.h
 *
 *  @date 11.11.2012
 *      @author Iwer Petersen
 */

#ifndef RGBDEMOCALIBRATION_H_
#define RGBDEMOCALIBRATION_H_

#include <pcl/common/eigen.h>
#include <opencv2/core/core.hpp>

/**
 * @class RGBDemoCalibration
 * @brief Loader for calibration file created by RGBDemo
 *
 *
 */
class RGBDemoCalibration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDemoCalibration(const char * cameraCalibrationFile,
            const char * projectorCalibrationFile);
    ~RGBDemoCalibration();
    /**
     * Camera Extrinsic getter
     * @return extrinsic matrix of the camera
     */
    Eigen::Matrix4f getCameraExtrinsics() const;

    /**
     * Camera Intrinsics getter
     * @return intrinsic matrix of the camera
     */
    Eigen::Matrix3f getCameraIntrinsics() const;

    /**
     * Projector Extrinsics getter
     * @return extrinsic matrix of the projector
     */
    Eigen::Matrix4f getProjectorExtrinsics() const;

    /**
     * Projector Intrinsics getter
     * @return intrinsic matrix of the projector
     */

    Eigen::Matrix3f getProjectorIntrinsics() const;
    /**
     * Projector Size getter
     * @return size of the projector
     */
    cv::Size getProjectorSize() const;

private:
    void loadCalibration();
    /**
     * load projector calibration
     */
    void loadProjectorCalibration();

    /**
     * load camera calibration
     */
    void loadCameraCalibration();

    /**
     * file name of camera calibration file
     */
    const char * cameraCalibrationFile_;

    /**
     * file name of projector calibration file
     */
    const char * projectorCalibrationFile_;

    /**
     * Camera Intrinsics
     */
    Eigen::Matrix3f cameraIntrinsics_;
    /**
     * Camera Extrinsics
     */
    Eigen::Matrix4f cameraExtrinsics_;

    /**
     * Projector Intrinsics
     */
    Eigen::Matrix3f projectorIntrinsics_;
    /**
     * Projector Extrinsics
     */
    Eigen::Matrix4f projectorExtrinsics_;
    /**
     *  Projector resolution X Y
     */
    cv::Size projectorSize_;

};

#endif /* RGBDEMOCALIBRATION_H_ */
