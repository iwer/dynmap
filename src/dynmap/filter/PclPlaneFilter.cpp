/**
 * @file PclPlaneFilter.cpp
 *
 *  @date 09.11.2012
 *      @author Iwer Petersen
 */

#include "PclPlaneFilter.h"

PclPlaneFilter::PclPlaneFilter() :
        coefficients_(new pcl::ModelCoefficients), inliers_(
                new pcl::PointIndices), seg_(), extract_() {
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    planeThreshold_ = 0.010f;
    planeremovalTime_ = 10;
}

PclPlaneFilter::~PclPlaneFilter() {
    // TODO Auto-generated destructor stub
}
void PclPlaneFilter::setInputCloud(const CloudConstPtr& cloud) {
    input_ = cloud;
}
void PclPlaneFilter::filter(Cloud& output) {
    // segment plane
    uint64 last_time = ntk::Time::getMillisecondCounter();
    seg_.setDistanceThreshold(planeThreshold_);

    if (input_->size() > 0) {
        seg_.setInputCloud(input_);
        seg_.segment(*inliers_, *coefficients_);
        if (inliers_->indices.size() > 0) {
            // extract segmented plane
            extract_.setInputCloud(input_);
            extract_.setIndices(inliers_);
            extract_.setNegative(true);
            extract_.filter(output);
        }
    }
    uint64 new_time = ntk::Time::getMillisecondCounter();
    planeremovalTime_ = (planeremovalTime_ + (new_time - last_time)) / 2;
    //    std::cout << "Plane Removal Time " << planTime_ << "ms" << std::endl;
}

float PclPlaneFilter::getPlaneremovalTime() const {
    return (planeremovalTime_);
}

float PclPlaneFilter::getPlaneThreshold() const {
    return (planeThreshold_);
}

void PclPlaneFilter::setPlaneThreshold(float planeThreshold) {
    planeThreshold_ = planeThreshold;
}

