/**
 * @file OpenGLFullScreen.h
 *
 *  @date 06.11.2012
 *      @author Iwer Petersen
 */

#ifndef OPENGLFULLSCREEN_H_
#define OPENGLFULLSCREEN_H_

#include "AppModel.h"
#include "AppController.h"
//#include "calibration/RGBDemoCalibration.h"
#include "common.h"
#include <QGLWidget>
#include <opencv2/core/core.hpp>
#include <pcl/common/transforms.h>

class AppController;
class AppModel;

namespace Ui {
class OpenGLFullscreen;
}

/**
 * @class OpenGLFullscreen
 * @brief The OpenGL Projector Window
 *
 *
 */
class OpenGLFullscreen: public QGLWidget {
Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit OpenGLFullscreen(AppModel * model, AppController * control,
            QWidget *parent = 0);
    ~OpenGLFullscreen();

    /**
     * set cloud to display
     * @param cloud
     */
    void setCloud(CloudConstPtr cloud);

protected slots:
    /**
     * update window
     */
    void updateContent();
//    void setXRotation(int angle);
//    void setYRotation(int angle);
//    void setZRotation(int angle);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
//    void mousePressEvent(QMouseEvent *event);
//    void mouseMoveEvent(QMouseEvent *event);

private:
    /**
     * User interface
     */
    Ui::OpenGLFullscreen *ui;
    /**
     * current point cloud
     */
    CloudPtr pointcloud_;
    /**
     * controller reference
     */
    AppController * control_;
    /**
     * model reference
     */
    AppModel * model_;
    /**
     * rot x
     */
    int xRot;
    /**
     * rot y
     */
    int yRot;
    /**
     * rot z
     */
    int zRot;
    /**
     * last pos TODO?
     */
    QPoint lastPos;
    /**
     * calibration loader TODO: get from model
     */
//    RGBDemoCalibration calib;
    /**
     * intrisics
     */
    Eigen::Matrix3f intr;
    /**
     * extrinsics
     */
    Eigen::Matrix4f extr;
    /**
     * projector size
     */
    cv::Size projSize;
    /**
     * projection matrix from intrinsics
     */
    GLfloat matrixProjection[16];
};

#endif // OPENGLFULLSCREEN_H
