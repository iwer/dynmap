/**
 * @file PclPlaneFilter.h
 *
 *  @date 09.11.2012
 *      @author Iwer Petersen
 */

#ifndef PCLPLANEFILTER_H_
#define PCLPLANEFILTER_H_

#include "common.h"
#include <ntk/utils/time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

/**
 * @class PclPlaneFilter
 * @brief PCL RANSAC Plane Extraction Wrapper
 *
 *
 */
 class PclPlaneFilter {
public:
    PclPlaneFilter();
    virtual ~PclPlaneFilter();
    /**
     * Set cloud to filter
     * @param cloud to filter
     */
    void setInputCloud (const CloudConstPtr &cloud);
    /**
     * apply filter
     * @param output cloud
     */
    void filter (Cloud &output);
    /**
     * Get filter time
     * @return filter time
     */
    float getPlaneremovalTime() const;
    /**
     * Get threshold
     * @return plane threshold
     */
    float getPlaneThreshold() const;
    /**
     * set threshold
     * @param planeThreshold
     */
    void setPlaneThreshold(float planeThreshold);

private:
    /**
     * the cloud to filter
     */
    CloudConstPtr input_;
    /**
     * plane threshold
     */
    float planeThreshold_;
    /**
     * filter time
     */
    float planeremovalTime_;

    /**
     * plane model coeffincients
     */
    pcl::ModelCoefficients::Ptr coefficients_;
    /**
     * inlier indices
     */
    pcl::PointIndices::Ptr inliers_;
    /**
     * segmentation instance
     */
    pcl::SACSegmentation<DefaultPoint> seg_;
    /**
     * extractor instance
     */
    pcl::ExtractIndices<DefaultPoint> extract_;

};

#endif /* PCLPLANEFILTER_H_ */
