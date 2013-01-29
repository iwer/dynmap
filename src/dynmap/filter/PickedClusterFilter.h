/**
 * @file PickedClusterFilter.h
 *
 *  @date 09.11.2012
 *      @author Iwer Petersen
 */

#ifndef PICKEDCLUSTERFILTER_H_
#define PICKEDCLUSTERFILTER_H_

#include "common.h"
#include <ntk/utils/time.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>



/**
 * @class PickedClusterFilter
 * @brief PCL Euclidean Cluster Wrapper
 *
 *
 */
 class PickedClusterFilter {
public:
    PickedClusterFilter();
    virtual ~PickedClusterFilter();
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
     * Set picked Point
     * @param picked point
     */
    void setPickedPoint(DefaultPoint p);
    /**
     * get filter time
     * @return filter time
     */
    float getClusterTime() const;
    /**
     * get cluster tolerance
     * @return cluster tolerance
     */
    float getClusterTolerance() const;
    /**
     * set cluster tolerance
     * @param clusterTolerance
     */
    void setClusterTolerance(float clusterTolerance);
    /**
     * get max size
     * @return max size
     */
    int getMaxSize() const;
    /**
     * set max size
     * @param maxSize
     */
    void setMaxSize(int maxSize);
    /**
     * get min size
     * @return min size
     */
    int getMinSize() const;
    /**
     * set min size
     * @param minSize
     */
    void setMinSize(int minSize);

private:
    /**
     * the cloud to filter
     */
    CloudConstPtr input_;
    /**
     * filter time
     */
    float clusterTime_;
    /**
     * picked point
     */
    DefaultPoint pickedPoint_;
    /**
     * cluster tolerance
     */
    float clusterTolerance_;
    /**
     * min points in cluster
     */
    int minSize_;
    /**
     * max points in cluster
     */
    int maxSize_;
    /**
     * clustered cloud
     */
    CloudPtr cluster_;
    /**
     * index of nearest points
     */
    std::vector<int> indices_;
    /**
     * distance to nearest points
     */
    std::vector<float> distances_;
    /**
     * cluster indices
     */
    std::vector<pcl::PointIndices> cluster_indices_;


    /**
     * Kdtree search object instance
     */
    pcl::search::KdTree<DefaultPoint>::Ptr searchTree_;
    /**
     * Euclidean Clusterer instance
     */
    pcl::EuclideanClusterExtraction<DefaultPoint> ec_;
};

#endif /* PICKEDCLUSTERFILTER_H_ */
