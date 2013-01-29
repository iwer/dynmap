/**
 *  @file CloudStorage.cpp
 *
 *  @date 07.10.2012
 *      @author Iwer Petersen
 */

#include "AppModelData.h"

AppModelData::AppModelData() :
        lastCloud_(new Cloud), lastImage_(new QImage), voxelCloud_(new Cloud), passThroughCloud_(
                new Cloud), planeRemovedCloud_(new Cloud), clusteredCloud_(
                new Cloud), lastModel_(new Cloud), models_(), models_it_(models_.end()) {

}

AppModelData::~AppModelData() {
    // TODO Auto-generated destructor stub
}

