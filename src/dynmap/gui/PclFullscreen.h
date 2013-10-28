#ifndef FULLSCREEN_H
#define FULLSCREEN_H

#include "AppController.h"
#include "AppModel.h"
#include <QWidget>
#include <QGLWidget>
#include <QRect>
#include <QApplication>
#include <QDesktopWidget>
#include <vtkCamera.h>
#include <pcl/visualization/pcl_visualizer.h>

class AppController;
class AppModel;

namespace Ui {
 class PclFullScreen;
}

/**
 * @class PclFullScreen
 * @brief The PCL Projector Window
 *
 *
 */
 class PclFullScreen: public QWidget {
Q_OBJECT

public:
     PclFullScreen(AppModel * model, AppController * control,
            QWidget *parent = 0);
    ~PclFullScreen();

protected:
    Ui::PclFullScreen *ui;

private:
    pcl::visualization::PCLVisualizer * cloudVisualizer_;
    AppController * control_;
    AppModel * model_;

private slots:
    void setCameraParameters(const Eigen::Matrix3f &intrinsics,
            const Eigen::Matrix4f &extrinsics);
    void updateContent();

};

#endif // FULLSCREEN_H
