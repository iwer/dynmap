/**
 * @file TrackingPipeline.cpp
 *
 *  @date 12.12.2012
 *      @author Iwer Petersen
 */

#include "filter/TrackingPipeline.h"

TrackingPipeline::TrackingPipeline(AppModel * model) :
        reference_(new Cloud), cloud_pass_downsampled_(new Cloud), model_(
                model), normalEstimate_(THREAD_NO), voxel_() {
    // setup normal estimator
    pcl::search::KdTree<DefaultPoint>::Ptr tree(
            new pcl::search::KdTree<DefaultPoint>(false));
    normalEstimate_.setSearchMethod(tree);
    normalEstimate_.setRadiusSearch(0.03);

    //setup tracker
    std::vector<double> default_step_covariance = std::vector<double>(6,
            0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double>(6,
            0.00001);
    std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

    boost::shared_ptr<
            pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<DefaultPoint,
                    ParticleType> > tracker(
            new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<DefaultPoint,
                    ParticleType>(THREAD_NO));
    maxParticles_ = 400;
    delta_ = 0.99;
    epsilon_ = 0.2;

    tracker->setMaximumParticleNum(maxParticles_);
    tracker->setDelta(delta_);
    tracker->setEpsilon(epsilon_);
    ParticleType bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;
    tracker->setBinSize(bin_size);

    tracker_ = tracker;
    tracker_->setTrans(Eigen::Affine3f::Identity());
    tracker_->setStepNoiseCovariance(default_step_covariance);
    tracker_->setInitialNoiseCovariance(initial_noise_covariance);
    tracker_->setInitialNoiseMean(default_initial_mean);
    tracker_->setIterationNum(1);

    unsigned int startParticles = maxParticles_ * 0.8;
    tracker_->setParticleNum(startParticles);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal(false);

    // setup coherences
    pcl::tracking::ApproxNearestPairPointCloudCoherence<DefaultPoint>::Ptr coherence =
            pcl::tracking::ApproxNearestPairPointCloudCoherence<DefaultPoint>::Ptr(
                    new pcl::tracking::ApproxNearestPairPointCloudCoherence<
                            DefaultPoint>());

    boost::shared_ptr<pcl::tracking::DistanceCoherence<DefaultPoint> > distance_coherence =
            boost::shared_ptr<pcl::tracking::DistanceCoherence<DefaultPoint> >(
                    new pcl::tracking::DistanceCoherence<DefaultPoint>());
    coherence->addPointCoherence(distance_coherence);

//    boost::shared_ptr<pcl::tracking::HSVColorCoherence<DefaultPoint> > color_coherence =
//            boost::shared_ptr<pcl::tracking::HSVColorCoherence<DefaultPoint> >(
//                    new pcl::tracking::HSVColorCoherence<DefaultPoint>());
//    color_coherence->setWeight(0.1);
//    coherence->addPointCoherence(color_coherence);

    boost::shared_ptr<pcl::search::KdTree<DefaultPoint> > search(
            new pcl::search::KdTree<DefaultPoint>(false));

    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(0.01);
    tracker_->setCloudCoherence(coherence);

    // tracking time initial guess;
    trackingTime_ = 20;
    downsampling_grid_size_ = 0.02;
}

TrackingPipeline::~TrackingPipeline() {
    // TODO Auto-generated destructor stub
}

void TrackingPipeline::start() {
    if (starter_mtx_.try_lock()) {
        worker_ = boost::thread(boost::bind(&TrackingPipeline::run, this));
        starter_mtx_.unlock();
    }
}

void TrackingPipeline::setReferenceModel(CloudPtr referenceModel) {
    Eigen::Vector4f centroid;
    // remove zero and nan points
    CloudPtr nonzero_ref(new Cloud);
    removeZeroPoints(referenceModel, *nonzero_ref);

    // compute centroid
    pcl::compute3DCentroid<DefaultPoint>(*nonzero_ref, centroid);

    // transform to centroid
    CloudPtr transed_ref(new Cloud);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
//    trans.translation().matrix() = Eigen::Vector3f(centroid[0], centroid[1],
//            centroid[2]);
    pcl::transformPointCloud<DefaultPoint>(*nonzero_ref, *transed_ref,
            trans.inverse());

    // downsample reference cloud
    reference_.reset(new Cloud);
    gridSample(transed_ref, *reference_, downsampling_grid_size_);

    // set reference model to tracker

    tracker_->setReferenceCloud(reference_);
    tracker_->setTrans(trans);
    tracker_->setMinIndices(int(referenceModel->points.size()) / 2);
}

void TrackingPipeline::removeZeroPoints(const CloudConstPtr &cloud,
        Cloud &result) {
    for (size_t i = 0; i < cloud->points.size(); i++) {
        DefaultPoint point = cloud->points[i];
        if (!(fabs(point.x) < 0.001 && fabs(point.y) < 0.001
                && fabs(point.z) < 0.001) && !pcl_isnan(point.x)
                && !pcl_isnan(point.y) && !pcl_isnan(point.z))
            result.points.push_back(point);
    }

    result.width = static_cast<uint32_t>(result.points.size());
    result.height = 1;
    result.is_dense = true;
}

void TrackingPipeline::gridSample(const CloudConstPtr &cloud, Cloud &result,
        double leaf_size) {
    voxel_.setLeafSize(float(leaf_size));
    voxel_.setInputCloud(cloud);
    voxel_.filter(result);
}

void TrackingPipeline::gridSampleApprox(const CloudConstPtr &cloud,
        Cloud &result, double leaf_size) {
    uint64 last_time = ntk::Time::getMillisecondCounter();
    grid.setLeafSize(static_cast<float>(leaf_size),
            static_cast<float>(leaf_size), static_cast<float>(leaf_size));
    grid.setInputCloud(cloud);
    grid.filter(result);
    //result = *cloud;
    uint64 new_time = ntk::Time::getMillisecondCounter();
    downsampling_time_ = (downsampling_time_ + (new_time - last_time)) / 2;
}

float TrackingPipeline::getDownsamplingGridSize() const {
    return (downsampling_grid_size_);
}

void TrackingPipeline::setDownsamplingGridSize(float downsamplingGridSize) {
    downsampling_grid_size_ = downsamplingGridSize;
}

void TrackingPipeline::setDelta(double delta) {
    delta_ = delta;
}

void TrackingPipeline::setEpsilon(double epsilon) {
    epsilon_ = epsilon;
}

void TrackingPipeline::setMaxParticles(unsigned int maxParticles) {
    maxParticles_ = maxParticles;
}

void TrackingPipeline::tracking(const CloudConstPtr &cloud) {
    uint64 last_time = ntk::Time::getMillisecondCounter();
    tracker_->setInputCloud(cloud);
    tracker_->compute();
    uint64 new_time = ntk::Time::getMillisecondCounter();
    trackingTime_ = (trackingTime_ + (new_time - last_time)) / 2;
}

void TrackingPipeline::run() {
    boost::mutex::scoped_lock lock(model_->data_.lastCloud_mtx_);
    boost::mutex::scoped_lock lock1(starter_mtx_);

    tracker_->setMaximumParticleNum(maxParticles_);
    tracker_->setDelta(delta_);
    tracker_->setEpsilon(epsilon_);

    cloud_pass_downsampled_.reset(new  Cloud);
    gridSampleApprox(model_->data_.lastCloud_, *cloud_pass_downsampled_,
            downsampling_grid_size_);
    tracking(cloud_pass_downsampled_);
    ParticleType result = tracker_->getResult();
    model_->data_.modelTransformation_ = tracker_->toEigenMatrix(result);
    model_->signalNewData();
}

float TrackingPipeline::getTrackingTime() const {
    return (trackingTime_);
}

float TrackingPipeline::getDownsamplingTime() const {
    return (downsampling_time_);
}

