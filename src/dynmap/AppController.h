/**
 *  @file AppController.h
 *
 *  @date 28.09.2012
 *      @author Iwer Petersen
 */

#ifndef APPCONTROLLER_H_
#define APPCONTROLLER_H_

#include "AppController.h"
//#include "I_Controller.h"
#include "AppModel.h"
#include "gui/mainwindow.h"
#include "common.h"
#include <QObject>
#include <QPixmap>
#include <QMetaType>
#include <ntk/utils/time.h>
#include <pcl/visualization/point_picking_event.h>




 class AppModel;
 class MainWindow;
 class I_Controller;

/**
 * @class AppController
 * @brief The MVC Controller
 *
 *
 */
 class AppController : public QObject {
     Q_OBJECT


public:
    AppController();
    virtual ~AppController();

public:
    /**
     * data updated function
     */
    void dataUpdated();
//    void selectedPoint(const pcl::visualization::PointPickingEvent &event);

protected:
    AppModel * model_;
    MainWindow * gui_;


signals:
    void newCloud();
    void newSmallCloud();
    void newRgbImage(QImage image);
    void newStatusMessage(QString message);

private:
//    AppModel * model_;
//    I_MainWindow * gui;

    /**
     * last update time
     */
    double lastTick_;
    /**
     * frame counter
     */
    int frameCounter_;
    /**
     * frameRate
     */
    float frameRate_;

    /**
     * update connection from model
     */
    boost::signals2::connection update_con_;
};

#endif /* APPCONTROLLER_H_ */
