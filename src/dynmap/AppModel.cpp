/**
 *  @file AppModel.cpp
 *
 *  @date 28.09.2012
 *      @author Iwer Petersen
 */

#include "AppModel.h"
#include <iostream>
#include <boost/thread.hpp>

bool initialized = false;

AppModel::AppModel() :
        processFilters(true), computeHull(false), displaymode_(
                DISPLAYMODE_RAW), opmode_(MODE_MODEL_ACQ), model_pipe_(
                new ModelAcquisitionPipeline(this)), track_pipe_(
                new TrackingPipeline(this)), writer_() {
    loadCalibration();
    pcl::PCDReader reader;
    CloudPtr model(new Cloud);
    reader.read("./reference-model.pcd", *model);
    data_.referenceTrackingModel_ = model;
    track_pipe_->setReferenceModel(data_.referenceTrackingModel_);

    grabber_ = new PclOpenNIGrabber(this);
//    grabber_->start();
    connectModelPipeline();
    initialized = true;
    std::cout << "Model initialized" << std::endl;
}

AppModel::~AppModel() {
    delete grabber_;
}

void AppModel::connectModelPipeline() {
    std::cout << "Connecting Acquisition Pipeline" << std::endl;
    boost::function<void()> model_pipe_cb = boost::bind(
            &ModelAcquisitionPipeline::start, model_pipe_);
    processing_con_ = update_sig_.connect(model_pipe_cb);
}

void AppModel::connectTrackingPipeline() {
    std::cout << "Connecting Tracking Pipeline" << std::endl;
    boost::function<void()> track_pipe_cb = boost::bind(
            &TrackingPipeline::start, track_pipe_);
    processing_con_ = update_sig_.connect(track_pipe_cb);
}

boost::signals2::connection AppModel::registerCallback(
        const boost::function<void()> callback) {
    boost::signals2::connection ret = updateFiltered_sig_.connect(callback);
    connections_.push_back(ret);
    return (ret);
}

void AppModel::signalNewData() {
    updateFiltered_sig_();
}

void AppModel::setProcessFilters(bool state) {
    processFilters = state;
}

void AppModel::setComputeHull(bool state) {
    computeHull = state;
}

float AppModel::getClusterTime() const {
    return (model_pipe_->cluster_.getClusterTime());
}

float AppModel::getPassTime() const {
    return (model_pipe_->pass_.getPassthroughTime());
}

float AppModel::getPlanTime() const {
    return (model_pipe_->planeRemove_.getPlaneremovalTime());
}

float AppModel::getVoxelTime() const {
    float time = 0;
    if (opmode_ == MODE_MODEL_ACQ) {
        time = model_pipe_->vox_.getVoxelgridTime();
    } else if (opmode_ == MODE_PERFORMANCE) {
        time = track_pipe_->getDownsamplingTime();
    }
    return (time);
}

int AppModel::getDisplaymode() const {
    return (displaymode_);
}

CloudPtr lastTransformedModel(new Cloud);
CloudPtr AppModel::getTransformedModel() {
    pcl::transformPointCloud<DefaultPoint>(*data_.referenceTrackingModel_,
            *lastTransformedModel, data_.modelTransformation_);
    return (lastTransformedModel);
}

void AppModel::setOperationMode(int mode) {
    if (mode == MODE_MODEL_ACQ) {
        opmode_ = mode;
        processing_con_.disconnect();
        connectModelPipeline();
    } else if (mode == MODE_PERFORMANCE) {
        opmode_ = mode;
        processing_con_.disconnect();
        connectTrackingPipeline();
    }
}

float AppModel::getTrackingTime() const {
    return (track_pipe_->getTrackingTime());
}

void AppModel::setParticleNo(int no) {
    track_pipe_->setMaxParticles(no);
}

void AppModel::setChiDelta(float delta) {
    track_pipe_->setDelta(delta);
}

void AppModel::setKLEpsilon(float epsilon) {
    track_pipe_->setEpsilon(epsilon);
}

void AppModel::loadCalibration() {
    calib_ = new RGBDemoCalibration("calibration-default.yml",
            "calibration-projector.yml");

    projector_intrinsics = calib_->getProjectorIntrinsics();
    projectorRT_ = calib_->getProjectorExtrinsics();

    Eigen::Affine3f rt(projectorRT_);
    projectorRTaffine_ = rt;
}

Eigen::Matrix4f AppModel::getProjectorRT() const {
    return (projectorRT_);
}

Eigen::Affine3f AppModel::getProjectorRTAffine() const {
    return (projectorRTaffine_);
}

Eigen::Matrix3f AppModel::getProjectorIntrinsics() const {
    return (projector_intrinsics);
}

int frame = 0;
void AppModel::setGrabberQImage(QImage * image) {
    if (frame > 30) {
        data_.lastImage_ = image;
    }
}

void AppModel::setGrabberPointCloud(CloudConstPtr cloud) {

    if (frame > 30) {
        {
            boost::mutex::scoped_lock lock(data_.lastCloud_mtx_);
            data_.lastCloud_ = cloud;
        }

        if (displaymode_ == DISPLAYMODE_RAW) {
            updateFiltered_sig_();
        }

        if (processFilters || opmode_ == MODE_PERFORMANCE) {
            update_sig_();
        }
    } else {
        frame++;
    }
}

QImage* AppModel::getLastImage() {
    return (data_.lastImage_);
}

float AppModel::getGrabberTime() const {
    return (grabber_->getFrameLength());
}

void AppModel::setDisplayMode(int mode) {
    displaymode_ = mode;
}

static int j = 0;
void AppModel::acceptModel() {
    data_.models_.push_back(data_.lastModel_);
    std::stringstream ss;
    ss << "model_cluster_" << j << ".pcd";
    writer_.write<DefaultPoint>(ss.str(), *data_.lastModel_, false);
    j++;
    data_.models_it_ = data_.models_.begin();
    data_.lastModel_.reset(new Cloud);
}

void AppModel::nextStoredModel() {
    data_.models_it_++;
    if (data_.models_it_ == data_.models_.end()) {
        data_.models_it_ = data_.models_.begin();
    }
}

void AppModel::previousStoredModel() {
    if (data_.models_it_ == data_.models_.begin()) {
        data_.models_it_ = data_.models_.end();
    }
    data_.models_it_--;
}

int AppModel::getStoredModelCount() {
    return (data_.models_.size());
}

void AppModel::pickPoint(const pcl::visualization::PointPickingEvent &event) {
    // pause processing
//    processFilters = false;

// set model iterator to end so that new model gets displayed
    data_.models_it_ = data_.models_.end();
    // extract point from PointPickEvent
    DefaultPoint picked_pt;
    event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);

    model_pipe_->setPickPoint(picked_pt);

//    // set data for clustering
//    {
//        boost::mutex::scoped_lock lock(data_.planeCloud_mtx_);
//
//        cluster_.setInputCloud(data_.planeRemovedCloud_);
//        cluster_.setPickedPoint(picked_pt);
//
//        cluster_.filter(*data_.lastModel_);
//    }

//    processFilters = true;
}

void AppModel::setPlaneThresh(float planeThresh) {
    model_pipe_->planeRemove_.setPlaneThreshold(planeThresh);
}

void AppModel::setLeafSize(float leafSize) {
        model_pipe_->vox_.setLeafSize(leafSize);
}

void AppModel::setTrackLeafSize(float leafSize) {
       track_pipe_->setDownsamplingGridSize(leafSize);
}

void AppModel::setXMax(float max) {
    model_pipe_->pass_.setMaxX(max);
}

void AppModel::setXMin(float min) {
    model_pipe_->pass_.setMinX(min);
}

void AppModel::setYMax(float max) {
    model_pipe_->pass_.setMaxY(max);
}

void AppModel::setYMin(float min) {
    model_pipe_->pass_.setMinY(min);
}

void AppModel::setZMax(float max) {
    model_pipe_->pass_.setMaxZ(max);
}

void AppModel::setZMin(float min) {
    model_pipe_->pass_.setMinZ(min);
}

CloudConstPtr last(new Cloud);
CloudConstPtr AppModel::getLastCloud() {
    if (opmode_ == MODE_MODEL_ACQ) {
        switch (displaymode_) {
        case DISPLAYMODE_PASS_FILTERED: {
            boost::mutex::scoped_lock lock(data_.passCloud_mtx_);
            last = data_.passThroughCloud_;
        }
            break;
        case DISPLAYMODE_VOXEL_FILTERED: {
            boost::mutex::scoped_lock lock(data_.voxelCloud_mtx_);
            last = data_.voxelCloud_;
        }
            break;
        case DISPLAYMODE_PLANE_FILTERED: {
            boost::mutex::scoped_lock lock(data_.planeCloud_mtx_);
            last = data_.planeRemovedCloud_;
        }
            break;
        case DISPLAYMODE_RAW:
        default: {
            boost::mutex::scoped_lock lock(data_.lastCloud_mtx_);
            data_.lastCloud_.swap(last);
        }
            break;

        }
    } else {
        last = data_.lastCloud_;
    }
    return (last);
}

CloudPtr AppModel::getLastModel() const {
    if (data_.models_it_ == data_.models_.end()) {
        return (data_.lastModel_);
    } else {
        return (*data_.models_it_);
    }
}
