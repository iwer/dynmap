/**
 * @file PclOpenNIGrabber.h
 *
 *  @date 09.11.2012
 *      @author Iwer Petersen
 */

#ifndef PCLOPENNIGRABBER_H_
#define PCLOPENNIGRABBER_H_

#include "common.h"
#include "AppModel.h"
#include <ntk/utils/time.h>
#include <pcl/io/openni_grabber.h>
#include <boost/signals2.hpp>
#include <QImage>

/**
 * @class PclOpenNIGrabber
 * @brief PCL OpenNIGrabber Wrapper
 *
 *
 */
class PclOpenNIGrabber {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ;

    PclOpenNIGrabber(AppModel * model);
    virtual ~PclOpenNIGrabber();

    void start();
    void stop();

    float getFrameLength();
    bool providesRGBQImage();
    bool providesPointCloud();

protected:
    /**
     * Callback for OpenNIGrabber cloud
     * @param cloud
     */
    void grabbedCloud_cb(const CloudConstPtr& cloud);
    /**
     * Callback for OpenNIGrabber image
     * @param image
     */
    void grabberImage_cb(const boost::shared_ptr<openni_wrapper::Image>& image);

private:
    /**
     * Model reference
     */
    AppModel * model_;
    /**
     * is initialized
     */
    bool initialized_;
    /**
     * initialize OpenNI
     */
    void initOpenNI();
    /**
     * needs to be friend
     */
    friend class pcl::OpenNIGrabber;
    /**
     * frame time
     */
    float frameLength_;
    /**
     * grabber instance
     */
    pcl::Grabber * grabber_;
    /**
     * cloud connection
     */
    boost::signals2::connection grabberCloud_con_;
    /**
     * image connection
     */
    boost::signals2::connection grabberImage_con_;

    /**
     * last cloud mutex
     */
    boost::mutex lastCloud_mtx_;
    /**
     * last image mutex
     */
    boost::mutex lastImage_mtx_;

    /**
     * last grabber cloud
     */
    CloudConstPtr lastCloud_;

    /**
     * last grabber image
     */
    boost::shared_ptr<openni_wrapper::Image> lastImage_;
    /**
     * rgb data
     */
    unsigned char* rgb_data_;
    /**
     * rgb data size
     */
    unsigned rgb_data_size_;
    /**
     * image width
     */
    unsigned imgWidth_;
    /**
     * image height
     */
    unsigned imgHeight_;
    /**
     * last QImage
     */
    QImage * qimage;

    /**
     * last frame time
     */
    uint64 last_time;
    /**
     * new frame time
     */
    uint64 new_time;
};

#endif /* PCLOPENNIGRABBER_H_ */
