/**
 * @file PclVoxelGridFilter.cpp
 *
 *  @date 08.11.2012
 *      @author Iwer Petersen
 */

#include "PclVoxelGridFilter.h"

PclVoxelGridFilter::PclVoxelGridFilter() : vox_() {
    leafSizeX_ = 0.02;
    leafSizeY_ = 0.02;
    leafSizeZ_ = 0.02;
    voxelgridTime_ = 10;
    vox_.setLeafSize(leafSizeX_, leafSizeY_, leafSizeZ_);
    vox_.setDownsampleAllData(true);
}

PclVoxelGridFilter::~PclVoxelGridFilter() {
    // TODO Auto-generated destructor stub
}
void PclVoxelGridFilter::setInputCloud(const CloudConstPtr& cloud) {
    input_ = cloud;
}
void PclVoxelGridFilter::filter(Cloud& output) {
    uint64 last_time = ntk::Time::getMillisecondCounter();
    vox_.setInputCloud(input_);
    vox_.setLeafSize(leafSizeX_, leafSizeY_, leafSizeZ_);
    vox_.filter(output);
    uint64 new_time = ntk::Time::getMillisecondCounter();
    voxelgridTime_ = (voxelgridTime_ + (new_time - last_time)) / 2;
    //    std::cout << "Voxel Time " << voxelTime_ << "ms" << std::endl;
}

void PclVoxelGridFilter::setLeafSize(float size) {
    leafSizeX_ = size;
    leafSizeY_ = size;
    leafSizeZ_ = size;
}

float PclVoxelGridFilter::getVoxelgridTime() const {
    return (voxelgridTime_);
}

