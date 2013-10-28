#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#include "gui/PclFullscreen.h"
#include "gui/OpenGLFullscreen.h"
#include "AppController.h"
#include "AppModel.h"
#include "common.h"
#include "displaymodes.h"
#include "operationmodes.h"
#include <vtkRenderWindow.h>
#include <QMainWindow>
#include <QRect>
#include <QApplication>
#include <QDesktopWidget>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <boost/signals2.hpp>

class AppController;
class AppModel;
class OpenGLFullscreen;

namespace Ui {
 class MainWindow;
}

/**
 * @class MainWindow
 * @brief The main window
 *
 *
 */
 class MainWindow: public QMainWindow {
Q_OBJECT

public:
    MainWindow(AppModel * model, AppController * control, QWidget *parent = 0);
    ~MainWindow();

    void registerPointPickCallback(boost::function<void (const pcl::visualization::PointPickingEvent&)> cb);
private:
    /**
     * User interface
     */
    Ui::MainWindow * ui;
    /**
     * Controller instance
     */
    AppController * control_;
    /**
     * Model instance
     */
    AppModel * model_;

    /**
     * main visualizer
     */
    pcl::visualization::PCLVisualizer * cloudVisualizer_;
    /**
     * small visualizer
     */
    pcl::visualization::PCLVisualizer * projectorVisualizer_;
    /**
     * point pick connection
     */
    boost::signals2::connection pp_con_;
    /**
     * value divider passtrough
     */
    float passValueDivider;
    /**
     * value divider voxelgrid
     */
    float voxelValueDivider;
    /**
     * value divider plane filter
     */
    float planeValueDivider;
    /**
     * fullscreen widget
     */
    OpenGLFullscreen * fullscreen;
    /**
     * render hull switch
     */
    bool renderHull;
    /**
     * application mode
     */
    int mode;

public slots:
    void updatePointCloud();
    void updateSmallPointCloud();
    void updateRgbImage(QImage pixmap);
    void updateStatusMessage(QString message);

private slots:
    void on_capturingCheckBox_toggled(bool checked);
    void on_passThroughRadioButton_clicked();
    void on_planeRemovalRadioButton_clicked();
    void on_resetCamera_clicked();
    void on_noFilterRadioButton_clicked();
    void on_voxelRadioButton_clicked();
    void on_passXMin_valueChanged(int value);
    void on_passXMax_valueChanged(int value);
    void on_passYMin_valueChanged(int value);
    void on_passYMax_valueChanged(int value);
    void on_passZMin_valueChanged(int value);
    void on_passZMax_valueChanged(int value);
    void on_voxelLeafSize_valueChanged(int value);
    void on_planeThresh_valueChanged(int value);
    void on_modeComboBox_currentIndexChanged(int index);
    void on_nextModelButton_clicked();
    void on_previousModelButton_clicked();
    void on_acceptModelButton_clicked();
    void on_actionShow_Camera_Position_toggled(bool arg1);
    void on_actionShow_Projector_Position_toggled(bool arg1);
    void on_trackLeafSize_valueChanged(int value);
    void on_chiDelta_valueChanged(int value);
    void on_epsilon_valueChanged(int value);
 };

#endif // MAINWINDOW_H

