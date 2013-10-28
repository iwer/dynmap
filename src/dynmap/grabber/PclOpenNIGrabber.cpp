/**
 * @file PclOpenNIGrabber.cpp
 *
 *  @date 09.11.2012
 *      @author Iwer Petersen
 */

#include "PclOpenNIGrabber.h"

PclOpenNIGrabber::PclOpenNIGrabber(AppModel * model) :
        model_(model), initialized_(false), lastCloud_(new Cloud), rgb_data_(
                0), rgb_data_size_(0), imgWidth_(640), imgHeight_(480) {
    frameLength_ = 33;
    initOpenNI();
    last_time = ntk::Time::getMillisecondCounter();
    initialized_ = true;
}

PclOpenNIGrabber::~PclOpenNIGrabber() {
    grabberCloud_con_.disconnect();
    grabberImage_con_.disconnect();
    grabber_->stop();
    while (grabber_->isRunning()) {
        sleep(1);
    }
    delete grabber_;
    if (rgb_data_) {
        delete[] rgb_data_;
    }
}

void PclOpenNIGrabber::initOpenNI() {
    // OpenNI Initialization
    std::string device_id("");
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_QVGA_30Hz;
    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_QVGA_30Hz;
//
//    openni_wrapper::OpenNIDriver& driver =
//            openni_wrapper::OpenNIDriver::getInstance();
//    if (driver.getNumberDevices() == 0) {
//        std::cout << "No OpenNI device connected." << std::endl;
//    }
//    std::cout
//            << "If a segfault occours here, OpenNI failed to startup because XnSensorServer "
//            << std::endl
//            << "instance still running in other process scope. Do :"
//            << std::endl << "killall XnSensorServer" << std::endl << std::endl;

    grabber_ = new pcl::OpenNIGrabber(device_id, depth_mode, image_mode);

    boost::function<void(const CloudConstPtr&)> raw_cloud_cb = boost::bind(
            &PclOpenNIGrabber::grabbedCloud_cb, this, _1);
    grabberCloud_con_ = grabber_->registerCallback(raw_cloud_cb);

    // image callback initialization
    boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&)> image_cb =
            boost::bind(&PclOpenNIGrabber::grabberImage_cb, this, _1);
    grabberImage_con_ = grabber_->registerCallback(image_cb);
    grabber_->start();
    while (!grabber_->isRunning()) {
        sleep(1);
    }
}

void PclOpenNIGrabber::grabbedCloud_cb(const CloudConstPtr& cloud) {
    if (initialized_) {
        new_time = ntk::Time::getMillisecondCounter();
        lastCloud_ = cloud;
        frameLength_ = (frameLength_ + (new_time - last_time)) / 2;

        model_->setGrabberPointCloud(lastCloud_);

        last_time = new_time;
    }
}

void PclOpenNIGrabber::grabberImage_cb(
        const boost::shared_ptr<openni_wrapper::Image>& image) {
    if (initialized_) {
        lastImage_ = image;
        // check if encoding is rgb (should be)
        if (image->getEncoding() != openni_wrapper::Image::RGB) {
            //resize data array
            if (rgb_data_size_ < image->getWidth() * image->getHeight()) {
                if (rgb_data_)
                    delete[] rgb_data_;
                rgb_data_size_ = image->getWidth() * image->getHeight();
                rgb_data_ = new unsigned char[rgb_data_size_ * 3];
            }
            // fill data
            lastImage_->fillRGB(image->getWidth(), image->getHeight(),
                    rgb_data_);
            // construct qImage
            qimage = new QImage(rgb_data_, image->getWidth(),
                    image->getHeight(), QImage::Format_RGB888);

            if (!qimage->isNull()) {
                // put into model
                model_->setGrabberQImage(qimage);
            }
        }
    }
}

float PclOpenNIGrabber::getFrameLength() {
    return (frameLength_);
}

bool PclOpenNIGrabber::providesRGBQImage() {
    return (true);
}

bool PclOpenNIGrabber::providesPointCloud() {
    return (true);
}

void PclOpenNIGrabber::start() {
}

void PclOpenNIGrabber::stop() {
    grabber_->stop();
}

