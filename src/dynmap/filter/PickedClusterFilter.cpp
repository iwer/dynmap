/**
 * @file PickedClusterFilter.cpp
 *
 *  @date 09.11.2012
 *      @author Iwer Petersen
 */

#include "PickedClusterFilter.h"

PickedClusterFilter::PickedClusterFilter() :
        indices_(1), distances_(1), searchTree_(
                new pcl::search::KdTree<DefaultPoint>), ec_() {
    clusterTime_ = 100;
    clusterTolerance_ = 0.05;
    minSize_ = 500;
    maxSize_ = 10000;

    ec_.setSearchMethod(searchTree_);

}

PickedClusterFilter::~PickedClusterFilter() {
    // TODO Auto-generated destructor stub
}

void PickedClusterFilter::setInputCloud(const CloudConstPtr& cloud) {
//    boost::mutex::scoped_lock lock(input_mtx_);
//    std::cout << "Changed input Cloud" << std::endl;
    input_ = cloud;
}

void PickedClusterFilter::filter(Cloud& output) {
    uint64 last_time = ntk::Time::getMillisecondCounter();
//    boost::mutex::scoped_lock lock(input_mtx_);

// Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
    searchTree_->setInputCloud(input_);
    searchTree_->nearestKSearch(pickedPoint_, 1, indices_, distances_);

    // euclidean clustering
    cluster_indices_.clear();
    ec_.setClusterTolerance(clusterTolerance_);
    ec_.setMinClusterSize(minSize_);
    ec_.setMaxClusterSize(maxSize_);
//    std::cout << "set:" << *input_ << std::endl;
    ec_.setInputCloud(input_);
    ec_.extract(cluster_indices_);

//    std::cout << "Picked: ";
//    for (int i = 0; i < indices_.size(); i++) {
//        std::cout << indices_[i];
//    }
//    std::cout << std::endl;

    int j = 0;
    // find picked point
    for (std::vector<pcl::PointIndices>::const_iterator cluster_it =
            cluster_indices_.begin(); cluster_it != cluster_indices_.end();
            ++cluster_it) {
//        std::cout << "Cluster " << j << ":" << std::endl;
//        for (std::vector<int>::const_iterator it = cluster_it->indices.begin(); it != cluster_it->indices.end(); it++) {
//            std::cout << *it << ", ";
//        }
//        std::cout << std::endl;

// if picked points index is in cluster
        if (std::find(cluster_it->indices.begin(), cluster_it->indices.end(),
                indices_[0]) != cluster_it->indices.end()) {

            CloudPtr cloud_cluster(new Cloud);
//            std::cout << "collect: "<< j << std::endl << *input_ << std::endl;

// collect all points in picked cluster
            for (std::vector<int>::const_iterator point_it =
                    cluster_it->indices.begin();
                    point_it != cluster_it->indices.end(); point_it++) {
                cloud_cluster->points.push_back(input_->points[*point_it]);
            }
            // set header
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            cloud_cluster->sensor_orientation_ = input_->sensor_orientation_;
            pcl::copyPointCloud<DefaultPoint>(*cloud_cluster, output);
        }
        j++;
    }
    uint64 new_time = ntk::Time::getMillisecondCounter();
    clusterTime_ = (clusterTime_ + (new_time - last_time)) / 2;
    //    std::cout << "Clustering Time " << clusterTime_ << "ms" << std::endl;
}

void PickedClusterFilter::setPickedPoint(DefaultPoint p) {
    pickedPoint_ = p;
}

float PickedClusterFilter::getClusterTime() const {
    return (clusterTime_);
}

float PickedClusterFilter::getClusterTolerance() const {
    return (clusterTolerance_);
}

void PickedClusterFilter::setClusterTolerance(float clusterTolerance) {
    clusterTolerance_ = clusterTolerance;
}

int PickedClusterFilter::getMaxSize() const {
    return (maxSize_);
}

void PickedClusterFilter::setMaxSize(int maxSize) {
    maxSize_ = maxSize;
}

int PickedClusterFilter::getMinSize() const {
    return (minSize_);
}

void PickedClusterFilter::setMinSize(int minSize) {
    minSize_ = minSize;
}

