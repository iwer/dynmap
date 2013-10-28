/**
 * @file ModelAcquisitionPipeline.cpp
 *
 *  @date 12.12.2012
 *      @author Iwer Petersen
 */

#include "filter/ModelAcquisitionPipeline.h"

ModelAcquisitionPipeline::ModelAcquisitionPipeline(AppModel * model) :
        model_(model), extract_cluster(false), vox_(), pass_(), planeRemove_(), cluster_() {

}

ModelAcquisitionPipeline::~ModelAcquisitionPipeline() {
}

void ModelAcquisitionPipeline::start() {
//    if (!extract_cluster) {
        worker_ = boost::thread(
                boost::bind(&ModelAcquisitionPipeline::run, this));
//    }
}

void ModelAcquisitionPipeline::run() {
    passthroughFilter();
    if (model_->getDisplaymode() == DISPLAYMODE_PASS_FILTERED) {
        model_->signalNewData();
    }
    voxelFilter();
    if (model_->getDisplaymode() == DISPLAYMODE_VOXEL_FILTERED) {
        model_->signalNewData();
    }
    planeRemove();
    if (model_->getDisplaymode() == DISPLAYMODE_PLANE_FILTERED) {
        model_->signalNewData();
    }
    if (extract_cluster) {
        extractCluster();
    }
}

void ModelAcquisitionPipeline::voxelFilter() {
    boost::mutex::scoped_lock lock1(model_->data_.passCloud_mtx_);
    vox_.setInputCloud(model_->data_.passThroughCloud_);
    {
        boost::mutex::scoped_lock lock2(model_->data_.voxelCloud_mtx_);
//        model_->data_.voxelCloud_.reset(new Cloud);
        vox_.filter(*model_->data_.voxelCloud_);
    }
}

void ModelAcquisitionPipeline::passthroughFilter() {
    boost::mutex::scoped_lock lock1(model_->data_.lastCloud_mtx_);
    pass_.setInputCloud(model_->data_.lastCloud_);
    {
        boost::mutex::scoped_lock lock2(model_->data_.passCloud_mtx_);
        model_->data_.passThroughCloud_.reset(new Cloud);
        pass_.filter(*model_->data_.passThroughCloud_);
    }
}

void ModelAcquisitionPipeline::planeRemove() {
    boost::mutex::scoped_lock lock1(model_->data_.voxelCloud_mtx_);

    planeRemove_.setInputCloud(model_->data_.voxelCloud_);
    {
        boost::mutex::scoped_lock lock2(model_->data_.planeCloud_mtx_);
        model_->data_.planeRemovedCloud_.reset(new Cloud);
        planeRemove_.filter(*model_->data_.planeRemovedCloud_);
    }
}

void ModelAcquisitionPipeline::setPickPoint(DefaultPoint p) {
    picked_pt = p;
    extract_cluster = true;
}

void ModelAcquisitionPipeline::extractCluster() {
    boost::mutex::scoped_lock lock(model_->data_.planeCloud_mtx_);

    cluster_.setInputCloud(model_->data_.planeRemovedCloud_);
    cluster_.setPickedPoint(picked_pt);

    cluster_.filter(*model_->data_.lastModel_);

    extract_cluster = false;
}

