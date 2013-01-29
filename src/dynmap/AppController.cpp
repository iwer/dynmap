/**
 *  @file AppController.cpp
 *
 *  @date 28.09.2012
 *      @author Iwer Petersen
 */

#include "AppController.h"
#include <QObject>

AppController::AppController() :
        lastTick_(0), frameCounter_(0), frameRate_(0) {
    model_ = new AppModel();
    gui_ = new MainWindow(model_, this);

    // register data update functions to model
    boost::function<void()> data_cb = boost::bind(&AppController::dataUpdated,
            this);
    update_con_ = model_->registerCallback(data_cb);

    // register pointpick callback of model to gui
    boost::function<void(const pcl::visualization::PointPickingEvent &)> pointpick_cb =
            boost::bind(&AppModel::pickPoint, model_, _1);
    gui_->registerPointPickCallback(pointpick_cb);

    // connect signal / slots to gui
    connect(this, SIGNAL(newCloud()), gui_, SLOT(updatePointCloud()));
    connect(this, SIGNAL(newSmallCloud(void)), gui_,
            SLOT(updateSmallPointCloud(void)));
    qRegisterMetaType<QImage>("QImage");
    connect(this, SIGNAL(newRgbImage(QImage)), gui_,
            SLOT(updateRgbImage(QImage)));
    qRegisterMetaType<QString>("QString");
    connect(this, SIGNAL(newStatusMessage(QString)), gui_,
            SLOT(updateStatusMessage(QString)));
    gui_->show();
}

AppController::~AppController() {
    update_con_.disconnect();
}

void AppController::dataUpdated() {
    ++frameCounter_;
    if (frameCounter_ == 10) {
        double current_tick = cv::getTickCount();
        frameRate_ = frameCounter_
                / ((current_tick - lastTick_) / cv::getTickFrequency());
        lastTick_ = current_tick;
        frameCounter_ = 0;
    }

    emit newSmallCloud();

    emit newCloud();

    QImage * image = model_->getLastImage();
    if (!image->isNull()) {
        QImage sImage = image->scaled(400, 300);
        emit newRgbImage(sImage);
    }

    QString status = QString("Final fps = %1 Grabber Time = %2 ms").arg(frameRate_,
            0, 'f', 1).arg(model_->getGrabberTime(), 0, 'f', 1);
    emit newStatusMessage(status);
}

