#include "PclFullscreen.h"
#include "ui_PclFullscreen.h"

PclFullScreen::PclFullScreen(AppModel * model, AppController * control,
        QWidget *parent) :
        QWidget(parent), ui(new Ui::PclFullScreen), cloudVisualizer_(
                new pcl::visualization::PCLVisualizer("Projector Viewer",
                        false)), control_(control), model_(model) {
    ui->setupUi(this);
    this->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);

    vtkSmartPointer<vtkRenderWindow> cloudRenderWindow =
            cloudVisualizer_->getRenderWindow();
    ui->projectorView->SetRenderWindow(cloudRenderWindow);
    cloudVisualizer_->setupInteractor(ui->projectorView->GetInteractor(),
            ui->projectorView->GetRenderWindow());
    cloudVisualizer_->getInteractorStyle()->setKeyboardModifier(
            pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
//    cloudVisualizer_->addCoordinateSystem(0.3);

// set "projector camera" intrinsics here?
    cloudVisualizer_->initCameraParameters();
//    cloudVisualizer_->setSize(1280, 1024);

    connect(control_, SIGNAL(newCloud()), this, SLOT(updateContent()));
}

PclFullScreen::~PclFullScreen() {
    delete ui;
}

void PclFullScreen::setCameraParameters(const Eigen::Matrix3f &intrinsics,
        const Eigen::Matrix4f &extrinsics) {
    int viewport = 0;

    // Position = extrinsic translation
    Eigen::Vector3f pos_vec = extrinsics.block<3, 1>(0, 3);

    // Rotate the view vector
    Eigen::Matrix3f rotation = extrinsics.block<3, 3>(0, 0);
    Eigen::Vector3f y_axis(0.f, 1.f, 0.f);
    Eigen::Vector3f up_vec(rotation * y_axis);

    // Compute the new focal point
    Eigen::Vector3f z_axis(0.f, 0.f, 1.f);
    Eigen::Vector3f focal_vec = pos_vec + rotation * z_axis;

    // Get the width and height of the image - assume the calibrated centers are at the center of the image
    Eigen::Vector2i window_size;
    window_size[0] = static_cast<int>(intrinsics(0, 2));
    window_size[1] = static_cast<int>(intrinsics(1, 2));

    // Compute the vertical field of view based on the focal length and image heigh
    double fovy = 2
            * atan((double) window_size[1] / (double) (2. * intrinsics(1, 1)))
            * 180.0 / M_PI;

    cloudVisualizer_->getRendererCollection()->InitTraversal();
    vtkRenderer* renderer = NULL;
    int i = 1;
    while ((renderer = cloudVisualizer_->getRendererCollection()->GetNextItem())
            != NULL) {
        // Modify all renderer's cameras
        if (viewport == 0 || viewport == i) {
            vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera();
            // This is what i would need to correct the perspective
            // But its only available in vtk-6.0.0
            //            cam->setUseOffAxisProjection(1);
            cam->SetPosition((double) pos_vec[0], (double) pos_vec[1],
                    (double) pos_vec[2]);
            cam->SetFocalPoint((double) focal_vec[0], (double) focal_vec[1],
                    (double) focal_vec[2]);
            cam->SetViewUp((double) up_vec[0], (double) up_vec[1],
                    (double) up_vec[2]);
            cam->SetUseHorizontalViewAngle(0);
            cam->SetViewAngle(fovy);
            cam->SetClippingRange(0.01, 1000.01);
            cloudVisualizer_->getRenderWindow()->SetSize((int) window_size[0],
                    (int) window_size[1]);

            renderer->Render();
        }
    }
}

void PclFullScreen::updateContent() {
//    pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>);
//    pcl::transformPointCloud(*cloud, *tmp_cloud,
//            AppModel::getInstance()->getProjectorRTAffine().inverse());

    if (!cloudVisualizer_->updatePointCloud(model_->getLastCloud(),
            "Projector Cloud")) {
        cloudVisualizer_->addPointCloud<DefaultPoint>(model_->getLastCloud(),
                "Projector Cloud");
        cloudVisualizer_->resetCameraViewpoint("Projector Cloud");
        this->setCameraParameters(model_->getProjectorIntrinsics(),
                model_->getProjectorRT());
        cloudVisualizer_->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5.0,
                "Projector Cloud");

    }
    ui->projectorView->update();
}
