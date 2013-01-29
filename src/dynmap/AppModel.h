/**
 *  @file AppModel.h
 *
 *  @date 28.09.2012
 *      @author Iwer Petersen
 */

#ifndef APPMODEL_H_
#define APPMODEL_H_

#include "AppModel.h"
#include "common.h"
#include "displaymodes.h"
#include "operationmodes.h"
#include "AppController.h"
#include "AppModelData.h"
#include "filter/PclPassThroughFilter.h"
#include "filter/PclVoxelGridFilter.h"
#include "filter/PclPlaneFilter.h"
#include "filter/PickedClusterFilter.h"
#include "filter/ModelAcquisitionPipeline.h"
#include "filter/TrackingPipeline.h"
#include "grabber/PclOpenNIGrabber.h"
#include "calibration/RGBDemoCalibration.h"

#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/point_picking_event.h>
#include <boost/signals2.hpp>

class ModelAcquisitionPipeline;
class TrackingPipeline;
class PclOpenNIGrabber;

/**
 * @class AppModel
 * @brief The MVC Model
 *
 *
 */
class AppModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AppModel();
    virtual ~AppModel();

    CloudPtr cloud_hull_;
    std::vector<pcl::Vertices> vertices_;

    boost::signals2::connection registerCallback(
            const boost::function<void()> callback);

    QImage * getLastImage();
    CloudConstPtr getLastCloud();
    CloudPtr getLastModel() const;
    CloudPtr getTransformedModel();

    // model acquisition filter parameters
    void setProcessFilters(bool state);
    void setComputeHull(bool state);
    void setPlaneThresh(float planeThresh);
    void setLeafSize(float leafSize);
    void setXMax(float max);
    void setXMin(float min);
    void setYMax(float max);
    void setYMin(float min);
    void setZMax(float max);
    void setZMin(float min);

    // tracking parameters
    void setTrackLeafSize(float leafSize);
    void setParticleNo(int no);
    void setChiDelta(float delta);
    void setKLEpsilon(float epsilon);


    // times
    float getGrabberTime() const;
    float getClusterTime() const;
    float getPassTime() const;
    float getPlanTime() const;
    float getVoxelTime() const;
    float getTrackingTime() const;

    // display mode
    int getDisplaymode() const;
    void setDisplayMode(int mode);
    void setOperationMode(int mode);

    void pickPoint(const pcl::visualization::PointPickingEvent &event);

    // captured model control
    int getStoredModelCount();
    void acceptModel();
    void nextStoredModel();
    void previousStoredModel();

    // transformations from calibration
    Eigen::Matrix4f getProjectorRT() const;
    Eigen::Affine3f getProjectorRTAffine() const;
    Eigen::Matrix3f getProjectorIntrinsics() const;

    // grabber callbacks
    void setGrabberQImage(QImage * image);
    void setGrabberPointCloud(CloudConstPtr cloud);

    // signal new data
    void signalNewData();

    AppModelData data_;
    RGBDemoCalibration * calib_;
    bool processFilters;

private:
    bool computeHull;
    int displaymode_;
    int opmode_;

    PclOpenNIGrabber * grabber_;
//    PickedClusterFilter cluster_;
    ModelAcquisitionPipeline* model_pipe_;
    TrackingPipeline* track_pipe_;

    boost::signals2::signal<void()> update_sig_;
    boost::signals2::signal<void()> updateFiltered_sig_;
    std::list<boost::signals2::connection> connections_;
    boost::signals2::connection processing_con_;
    pcl::PCDWriter writer_;
    Eigen::Matrix4f projectorRT_;
    Eigen::Matrix3f projectorR_;
    Eigen::Matrix3f projectorT_;
    Eigen::Affine3f projectorRTaffine_;
    Eigen::Matrix3f projector_intrinsics;
    void connectModelPipeline();
    void connectTrackingPipeline();
    void loadCalibration();
};

#endif /* APPMODEL_H_ */
