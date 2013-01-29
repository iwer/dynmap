/**
 * @file TrackingPipeline.h
 *
 *  @date 12.12.2012
 *      @author Iwer Petersen
 */

#ifndef TRACKINGPIPELINE_H_
#define TRACKINGPIPELINE_H_

#include "AppModel.h"
#include "common.h"
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/common/centroid.h>
#include <pcl/pcl_macros.h>

#include <boost/thread.hpp>


#define THREAD_NO 8

/**
 * @class TrackingPipeline
 * @brief Filter Pipeline for Tracking
 *
 *
 */
 class TrackingPipeline {
    typedef pcl::tracking::ParticleXYZRPY ParticleType;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrackingPipeline(AppModel * model);
    virtual ~TrackingPipeline();
    void start();
    void setReferenceModel(CloudPtr referenceModel);
    float getTrackingTime() const;
    float getDownsamplingTime() const;
    float getDownsamplingGridSize() const;
    void setDownsamplingGridSize(float downsamplingGridSize);
    void setDelta(double delta);
    void setEpsilon(double epsilon);
    void setMaxParticles(unsigned int maxParticles);

private:
    void run();
    void removeZeroPoints(const CloudConstPtr& cloud, Cloud& result);
    void gridSample(const CloudConstPtr& cloud, Cloud& result, double leaf_size = 0.01);
    void gridSampleApprox(const CloudConstPtr& cloud, Cloud& result, double leaf_size = 0.01);
    void tracking(const CloudConstPtr& cloud);
    CloudPtr reference_;
    CloudPtr cloud_pass_downsampled_;
    AppModel* model_;
    boost::thread worker_;
    unsigned int maxParticles_;
    double delta_;
    double epsilon_;
    float downsampling_grid_size_;
    pcl::NormalEstimationOMP<DefaultPoint,pcl::Normal> normalEstimate_;
    boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<DefaultPoint,ParticleType> > tracker_;
    float trackingTime_;
    float downsampling_time_;
    boost::mutex starter_mtx_;
    pcl::ApproximateVoxelGrid<DefaultPoint> grid;

public:
    PclVoxelGridFilter voxel_;
};

#endif /* TRACKINGPIPELINE_H_ */
