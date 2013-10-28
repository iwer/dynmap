/**
 * @file PclVoxelGridFilter.h
 *
 *  @date 08.11.2012
 *      @author Iwer Petersen
 */

#ifndef PCLVOXELGRIDFILTER_H_
#define PCLVOXELGRIDFILTER_H_

#include "common.h"
#include <ntk/utils/time.h>
#include <pcl/filters/voxel_grid.h>

/**
 * @class PclVoxelGridFilter
 * @brief PCL Voxelgrid Wrapper
 *
 *
 */
class PclVoxelGridFilter {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    PclVoxelGridFilter();
    ~PclVoxelGridFilter();
    /**
     * Set cloud to filter
     * @param cloud to filter
     */
    void setInputCloud(const CloudConstPtr &cloud);
    /**
     * apply filter
     * @param output cloud
     */
    void filter(Cloud &output);

    /**
     * set uniform leafsize
     * @param size
     */
    void setLeafSize(float size);
    /**
     * get filter time
     * @return filter time
     */
    float getVoxelgridTime() const;

private:
    /**
     * the cloud to filter
     */
    CloudConstPtr input_;
    /**
     * leaf size x
     */
    float leafSizeX_;
    /**
     * leaf size y
     */
    float leafSizeY_;
    /**
     * leaf size z
     */
    float leafSizeZ_;
    /**
     * filter time
     */
    float voxelgridTime_;

    /**
     * voxel filter instance
     */
    pcl::VoxelGrid<DefaultPoint> vox_;

};

#endif /* PCLVOXELGRIDFILTER_H_ */
