/**
 * @file PclPassThroughFilter.h
 *
 *  @date 08.11.2012
 *      @author Iwer Petersen
 */

#ifndef PCLPASSTHROUGHFILTER_H_
#define PCLPASSTHROUGHFILTER_H_

#include "common.h"
#include <ntk/utils/time.h>
#include <pcl/filters/passthrough.h>
/*
 *
 */
/**
 * @class PclPassThroughFilter
 * @brief PCL Passthrough Wrapper
 *
 *
 */
 class PclPassThroughFilter {
public:
    PclPassThroughFilter();
    virtual ~PclPassThroughFilter();

    /**
     * Set cloud to filter
     * @param cloud to filter
     */
    void setInputCloud (const CloudConstPtr &cloud);
    /**
     * apply filter
     * @param output cloud
     */
    void filter (Cloud &output);
    /**
     * Get x-axis maximum
     * @return x-axis maximum
     */
    float getMaxX() const;
    /**
     * Set x-axis maximum
     * @param maxX
     */
    void setMaxX(float maxX);
    /**
     * Get y-axis maximum
     * @return y-axis maximum
     */
    float getMaxY() const;
    /**
     * Set y-axis maximum
     * @param maxY
     */
    void setMaxY(float maxY);
    /**
     * Get z-axis maximum
     * @return z-axis maximum
     */
    float getMaxZ() const;
    /**
     * Set z-axis maximum
     * @param maxZ
     */
    void setMaxZ(float maxZ);
    /**
     * Get x-axis minimum
     * @return x-axis minimum
     */
    float getMinX() const;
    /**
     * Set x-axis minimum
     * @param minX
     */
    void setMinX(float minX);
    /**
     * Get y-axis minimum
     * @return y-axis minimum
     */
    float getMinY() const;
    /**
     * Set y-axis minimum
     * @param minY
     */
    void setMinY(float minY);
    /**
     * Get z-axis minimum
     * @return z-axis minimum
     */
    float getMinZ() const;
    /**
     * Set z-axis minimum
     * @param minZ
     */
    void setMinZ(float minZ);
    /**
     * Get filter time
     * @return filter time
     */
    float getPassthroughTime() const;


private:
    /**
     * the cloud to filter
     */
    CloudConstPtr input_;
    /**
     * x min
     */
    float MinX_;
    /**
     * x max
     */
    float MaxX_;
    /**
     * y min
     */
    float MinY_;
    /**
     * y max
     */
    float MaxY_;
    /**
     * z min
     */
    float MinZ_;
    /**
     * z max
     */
    float MaxZ_;
    /**
     * filter time
     */
    float passthroughTime_;

    /**
     * X Filter
     */
    pcl::PassThrough<DefaultPoint> pass_x_;
    /**
     * Y Filter
     */
    pcl::PassThrough<DefaultPoint> pass_y_;
    /**
     * Z Filter
     */
    pcl::PassThrough<DefaultPoint> pass_z_;
    /**
     * Cloud after z filtering
     */
    CloudPtr zFiltered_;
    /**
     * Cloud after y filtering
     */
    CloudPtr yFiltered_;

};

#endif /* PCLPASSTHROUGHFILTER_H_ */
