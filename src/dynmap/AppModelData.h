/**
 *  @file AppModelData.h
 *
 *  @date 07.10.2012
 *      @author Iwer Petersen
 */

#ifndef CLOUDSTORAGE_H_
#define CLOUDSTORAGE_H_

#include "common.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <QImage>
#include <boost/thread.hpp>

/**
 * @class AppModelData
 * @brief Holds Cloud data
 *
 *
 */
 class AppModelData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AppModelData();
    virtual ~AppModelData();

    CloudConstPtr lastCloud_;
    QImage * lastImage_;

    // intermediate states
    CloudPtr voxelCloud_;
    CloudPtr passThroughCloud_;
    CloudPtr planeRemovedCloud_;
    CloudPtr clusteredCloud_;

    // for model acquisition
    CloudPtr lastModel_;
    std::vector<CloudPtr> models_;
    std::vector<CloudPtr>::const_iterator models_it_;

    boost::mutex lastCloud_mtx_;
    boost::mutex passCloud_mtx_;
    boost::mutex voxelCloud_mtx_;
    boost::mutex planeCloud_mtx_;
    boost::mutex lastImage_mtx_;

    // for tracking
    CloudPtr referenceTrackingModel_;
    Eigen::Affine3f modelTransformation_;
};

#endif /* CLOUDSTORAGE_H_ */
