#include "gui/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(AppModel * model, AppController * control,
        QWidget *parent) :
        ui(new Ui::MainWindow), control_(control), model_(model), cloudVisualizer_(
                new pcl::visualization::PCLVisualizer("Cloud Viewer", false)), projectorVisualizer_(
                new pcl::visualization::PCLVisualizer("Model Viewer", false)), passValueDivider(
                100), voxelValueDivider(1000), planeValueDivider(1000) {
    ui->setupUi(this);
    this->mode = MODE_MODEL_ACQ;
    fullscreen = new OpenGLFullscreen(model, control, this);

    // setup fullscreen
    this->move(QPoint(0, 0));
    renderHull = false;
    QRect screenres = QApplication::desktop()->screenGeometry(
            1/*screenNumber*/);
    fullscreen->setAttribute(Qt::WA_DeleteOnClose, true);
//    fullscreen.show();
    fullscreen->move(QPoint(screenres.x(), screenres.y()));
    fullscreen->resize(screenres.width(), screenres.height());
    fullscreen->hide();
    std::cout << "Started Projector Window : " << screenres.width() << "x"
            << screenres.height() << " at " << screenres.x() << ","
            << screenres.y() << std::endl;

    // setup main cloud visualizer
    vtkSmartPointer<vtkRenderWindow> cloudRenderWindow =
            cloudVisualizer_->getRenderWindow();
    ui->pclView->SetRenderWindow(cloudRenderWindow);

    cloudVisualizer_->setupInteractor(ui->pclView->GetInteractor(),
            ui->pclView->GetRenderWindow());
    cloudVisualizer_->getInteractorStyle()->setKeyboardModifier(
            pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    cloudVisualizer_->setBackgroundColor(1.0, 1.0, 1.0);
    cloudVisualizer_->addCoordinateSystem(0.2);
    cloudVisualizer_->addCoordinateSystem(0.3, model_->getProjectorRTAffine());

// setup small cloud visualizer
    vtkSmartPointer<vtkRenderWindow> projectorWindow =
            projectorVisualizer_->getRenderWindow();
    ui->projectorView->SetRenderWindow(projectorWindow);
    projectorVisualizer_->setBackgroundColor(0.8, 0.8, 0.8);
}

MainWindow::~MainWindow() {
    pp_con_.disconnect();
    fullscreen->hide();
    fullscreen->close();
    delete control_;
    delete ui;
}

void MainWindow::on_voxelRadioButton_clicked() {
    model_->setDisplayMode(DISPLAYMODE_VOXEL_FILTERED);
}

void MainWindow::on_passThroughRadioButton_clicked() {
    model_->setDisplayMode(DISPLAYMODE_PASS_FILTERED);
}

void MainWindow::on_planeRemovalRadioButton_clicked() {
    model_->setDisplayMode(DISPLAYMODE_PLANE_FILTERED);
}

void MainWindow::on_resetCamera_clicked() {
    cloudVisualizer_->resetCameraViewpoint("Tech Cloud");
    projectorVisualizer_->resetCameraViewpoint("Projector Cloud Helper");
}

void MainWindow::on_noFilterRadioButton_clicked() {
    model_->setDisplayMode(DISPLAYMODE_RAW);
}

void MainWindow::updatePointCloud() {
//    FPS_CALC_BEGIN;
    CloudConstPtr localCloud = model_->getLastCloud();
//    if (!cloudVisualizer_->updatePointCloud(model_->getLastCloud(), "Tech Cloud")) {
    if (!cloudVisualizer_->updatePointCloud(localCloud, "Tech Cloud")) {
//        cloudVisualizer_->addPointCloud<DefaultPoint>(model_->getLastCloud(), "Tech Cloud");
        cloudVisualizer_->addPointCloud<DefaultPoint>(localCloud, "Tech Cloud");
        cloudVisualizer_->resetCameraViewpoint("Tech Cloud");
    }
    ui->pclView->update();

//    fullscreen->updateContent();

    if (mode == MODE_PERFORMANCE) {
        CloudPtr model = model_->getTransformedModel();
        if (!cloudVisualizer_->updatePointCloud<DefaultPoint>(model,
                "Model Cloud")) {
            //        cloudVisualizer_->addPointCloud<DefaultPoint>(model_->getLastCloud(), "Tech Cloud");
            cloudVisualizer_->addPointCloud<DefaultPoint>(model, "Model Cloud");
        }
        fullscreen->setCloud(model);
//        fullscreen->updateContent();
    }

    QString passTime = QString("Passthrough Time: %1 ms").arg(
            model_->getPassTime(), 5, 'f', 2);
    QString voxTime = QString("Voxel Time: %1 ms").arg(model_->getVoxelTime(),
            5, 'f', 2);
    QString planeTime = QString("Plane Removal Time: %1 ms").arg(
            model_->getPlanTime(), 5, 'f', 2);
    QString clusterTime = QString("Cluster Time: %1 ms").arg(
            model_->getClusterTime(), 5, 'f', 2);
    QString trackingTime = QString("Tracking Time: %1 ms").arg(
            model_->getTrackingTime(), 5, 'f', 2);

    ui->passTimeLabel->setText(passTime);
    ui->voxelTimeLabel->setText(voxTime);
    ui->planeTimeLabel->setText(planeTime);
    ui->clusterTimeLabel->setText(clusterTime);
    ui->trackingTimeLabel->setText(trackingTime);
}

void MainWindow::updateSmallPointCloud() {
    CloudConstPtr localCloud = model_->getLastModel();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(localCloud);
    if (mode == MODE_MODEL_ACQ) {
        if (!projectorVisualizer_->updatePointCloud<DefaultPoint>(
                localCloud, rgb, "Model Cloud")) {
            projectorVisualizer_->addPointCloud<DefaultPoint>(
                    localCloud, rgb, "Model Cloud");
        }
        ui->projectorView->update();
    }
}

//TODO: No argument needed, can get Cloud directly from Model
void MainWindow::updateRgbImage(QImage pixmap) {
    ui->rgbView->setPixmap(QPixmap::fromImage(pixmap));
}

void MainWindow::updateStatusMessage(QString message) {
    ui->statusBar->showMessage(message);
}

void MainWindow::on_capturingCheckBox_toggled(bool checked) {
    model_->setProcessFilters(checked);
}

void MainWindow::on_passXMin_valueChanged(int value) {
    float passXMin = value / passValueDivider;
    model_->setXMin(passXMin);
    QString text = QString("%1").arg(passXMin, 3, 'f', 2);
    ui->passXMinValue->setText(text);
}

void MainWindow::on_passXMax_valueChanged(int value) {
    float passXMax = value / passValueDivider;
    model_->setXMax(passXMax);
    QString text = QString("%1").arg(passXMax, 3, 'f', 2);
    ui->passXMaxValue->setText(text);
}

void MainWindow::on_passYMin_valueChanged(int value) {
    float passYMin = value / passValueDivider;
    model_->setYMin(passYMin);
    QString text = QString("%1").arg(passYMin, 3, 'f', 2);
    ui->passYMinValue->setText(text);
}

void MainWindow::on_passYMax_valueChanged(int value) {
    float passYMax = value / passValueDivider;
    model_->setYMax(passYMax);
    QString text = QString("%1").arg(passYMax, 3, 'f', 2);
    ui->passYMaxValue->setText(text);

}

void MainWindow::on_passZMin_valueChanged(int value) {
    float passZMin = value / passValueDivider;
    model_->setZMin(passZMin);
    QString text = QString("%1").arg(passZMin, 3, 'f', 2);
    ui->passZMinValue->setText(text);

}

void MainWindow::on_passZMax_valueChanged(int value) {
    float passZMax = value / passValueDivider;
    model_->setZMax(passZMax);
    QString text = QString("%1").arg(passZMax, 3, 'f', 2);
    ui->passZMaxValue->setText(text);

}

void MainWindow::on_voxelLeafSize_valueChanged(int value) {
    float voxelLeafSize = value / voxelValueDivider;
    model_->setLeafSize(voxelLeafSize);
    QString text = QString("%1").arg(voxelLeafSize, 3, 'f', 3);
    ui->voxelLeafSizeValue->setText(text);

}

void MainWindow::on_planeThresh_valueChanged(int value) {
    float planeThresh = value / planeValueDivider;
    model_->setPlaneThresh(planeThresh);
    QString text = QString("%1").arg(planeThresh, 3, 'f', 3);
    ui->planeThreshValue->setText(text);

}

void MainWindow::registerPointPickCallback(
        boost::function<void(const pcl::visualization::PointPickingEvent&)> cb) {
    pp_con_ = cloudVisualizer_->registerPointPickingCallback(cb);

}

void MainWindow::on_modeComboBox_currentIndexChanged(int index) {
    if (index == MODE_MODEL_ACQ) {
        mode = MODE_MODEL_ACQ;
        model_->setOperationMode(mode);
        fullscreen->hide();
        cloudVisualizer_->removePointCloud("Model Cloud");
        projectorVisualizer_->removeAllPointClouds();
    } else if (index == MODE_PERFORMANCE) {
        mode = MODE_PERFORMANCE;
        model_->setOperationMode(mode);
        fullscreen->showFullScreen();
        projectorVisualizer_->removeAllPointClouds();
    }
}

void MainWindow::on_nextModelButton_clicked() {
    model_->nextStoredModel();
}

void MainWindow::on_previousModelButton_clicked() {
    model_->previousStoredModel();
}

void MainWindow::on_acceptModelButton_clicked() {
    model_->acceptModel();
    ui->nextModelButton->setEnabled(true);
    ui->previousModelButton->setEnabled(true);
    QString modelInfo = QString("Models: %1").arg(
            model_->getStoredModelCount());
    ui->modelCountLabel->setText(modelInfo);
}

void MainWindow::on_actionShow_Camera_Position_toggled(bool state) {

}

void MainWindow::on_actionShow_Projector_Position_toggled(bool state) {

}

void MainWindow::on_trackLeafSize_valueChanged(int value) {
    float voxelLeafSize = value / voxelValueDivider;
    model_->setTrackLeafSize(voxelLeafSize);
    QString text = QString("%1").arg(voxelLeafSize, 3, 'f', 3);
    ui->trackLeafValue->setText(text);
}

void MainWindow::on_chiDelta_valueChanged(int value) {
    float chiDelta = value / passValueDivider;
    model_->setChiDelta(chiDelta);
    QString text = QString("%1").arg(chiDelta, 3, 'f', 2);
    ui->chiDeltaValue->setText(text);
}

void MainWindow::on_epsilon_valueChanged(int value) {
    float epsilon = value / passValueDivider;
    model_->setKLEpsilon(epsilon);
    QString text = QString("%1").arg(epsilon, 3, 'f', 2);
    ui->epsilonValue->setText(text);
}
