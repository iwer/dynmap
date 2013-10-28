/**
 * @file ModelAcquisitionPipeline.h
 *
 *  @date 12.12.2012
 *      @author Iwer Petersen
 */

#ifndef MODELACQUISITIONPIPELINE_H_
#define MODELACQUISITIONPIPELINE_H_

#include "AppModel.h"
#include "filter/PclPassThroughFilter.h"
#include "filter/PclVoxelGridFilter.h"
#include "filter/PclPlaneFilter.h"
#include "filter/PickedClusterFilter.h"
#include <boost/thread.hpp>


 class AppModel;

/**
 * @class ModelAcquisitionPipeline
 * @brief Filter Pipeline for Model Acquisition
 *
 *
 */
 class ModelAcquisitionPipeline {
public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ModelAcquisitionPipeline(AppModel * model);
    virtual ~ModelAcquisitionPipeline();

    /**
     * Start pipeline
     */
    void start();
    /**
     * Set picked point
     * @param picked point
     */
    void setPickPoint(DefaultPoint p);

private:
    /**
     * thread function
     */
    void run();
    /**
     * voxel filter function
     */
    void voxelFilter();
    /**
     * passthrough filter function
     */
    void passthroughFilter();
    /**
     * plane extraction filter function
     */
    void planeRemove();
    /**
     * cluster extraction filter function
     */
    void extractCluster();

    /**
     * reference to model
     */
    AppModel * model_;
    /**
     * worker thread
     */
    boost::thread worker_;
    /**
     * picked point
     */
    DefaultPoint picked_pt;
    /**
     * extract switch
     */
    bool extract_cluster;

public:
    /**
     * Voxel Filter instance
     */
    PclVoxelGridFilter vox_;
    /**
     * Pass Filter instance
     */
    PclPassThroughFilter pass_;
    /**
     * Plane Filter instance
     */
    PclPlaneFilter planeRemove_;
    /**
     * Cluster Filter instance
     */
    PickedClusterFilter cluster_;

};

#endif /* MODELACQUISITIONPIPELINE_H_ */
