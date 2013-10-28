/**
 * @file PclPassThroughFilter.cpp
 *
 *  @date 08.11.2012
 *      @author Iwer Petersen
 */

#include "PclPassThroughFilter.h"

PclPassThroughFilter::PclPassThroughFilter() :
        pass_x_(), pass_y_(), pass_z_(), zFiltered_(new Cloud), yFiltered_(
                new Cloud) {
    MinX_ = -1.0f;
    MaxX_ = 1.0f;
    MinY_ = -0.6f;
    MaxY_ = 0.6f;
    MinZ_ = 0.5f;
    MaxZ_ = 1.5f;
    pass_x_.setFilterFieldName("x");
    pass_x_.setKeepOrganized(true);
    pass_y_.setFilterFieldName("y");
    pass_y_.setKeepOrganized(true);
    pass_z_.setFilterFieldName("z");
    pass_z_.setKeepOrganized(true);
    // initial guess for filter time
    passthroughTime_ = 10;
}

PclPassThroughFilter::~PclPassThroughFilter() {
    // TODO Auto-generated destructor stub
}
void PclPassThroughFilter::setInputCloud(const CloudConstPtr& cloud) {
    input_ = cloud;
}

void PclPassThroughFilter::filter(Cloud& output) {
    // get start time
    uint64 last_time = ntk::Time::getMillisecondCounter();
    // reset filter limits
    pass_x_.setFilterLimits(MinX_, MaxX_);
    pass_y_.setFilterLimits(MinY_, MaxY_);
    pass_z_.setFilterLimits(MinZ_, MaxZ_);
    // filter through the 3 stages
    pass_z_.setInputCloud(input_);
    pass_z_.filter(*zFiltered_);
    pass_y_.setInputCloud(zFiltered_);
    pass_y_.filter(*yFiltered_);
    pass_x_.setInputCloud(yFiltered_);
    pass_x_.filter(output);
    //stop time
    uint64 new_time = ntk::Time::getMillisecondCounter();
    passthroughTime_ = (passthroughTime_ + (new_time - last_time)) / 2;
}

float PclPassThroughFilter::getMaxX() const {
    return (MaxX_);
}

void PclPassThroughFilter::setMaxX(float maxX) {
    MaxX_ = maxX;
}

float PclPassThroughFilter::getMaxY() const {
    return (MaxY_);
}

void PclPassThroughFilter::setMaxY(float maxY) {
    MaxY_ = maxY;
}

float PclPassThroughFilter::getMaxZ() const {
    return (MaxZ_);
}

void PclPassThroughFilter::setMaxZ(float maxZ) {
    MaxZ_ = maxZ;
}

float PclPassThroughFilter::getMinX() const {
    return (MinX_);
}

void PclPassThroughFilter::setMinX(float minX) {
    MinX_ = minX;
}

float PclPassThroughFilter::getMinY() const {
    return (MinY_);
}

void PclPassThroughFilter::setMinY(float minY) {
    MinY_ = minY;
}

float PclPassThroughFilter::getMinZ() const {
    return (MinZ_);
}

void PclPassThroughFilter::setMinZ(float minZ) {
    MinZ_ = minZ;
}

float PclPassThroughFilter::getPassthroughTime() const {
    return (passthroughTime_);
}

