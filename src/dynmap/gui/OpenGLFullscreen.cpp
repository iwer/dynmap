#include "OpenGLFullscreen.h"
#include "ui_OpenGLFullscreen.h"
#include <QtGui>
#include <QtOpenGL>
#include <GL/glu.h>
#include <pcl/common/io.h>

OpenGLFullscreen::OpenGLFullscreen(AppModel * model, AppController * control,
        QWidget *parent) :
        QGLWidget(parent), ui(new Ui::OpenGLFullscreen), control_(control), model_(
                model)/*, calib("calibration-default.yml",
                "calibration-projector.yml")*/ {
    //TODO get calib from model
    ui->setupUi(this);
    this->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
    projSize = model_->calib_->getProjectorSize();
//    projSize = calib.getProjectorSize();
    intr = model_->calib_->getProjectorIntrinsics();
//    intr = calib.getProjectorIntrinsics();
    extr = model_->calib_->getProjectorExtrinsics().inverse();
//    extr = calib.getProjectorExtrinsics().inverse();
    xRot = -90 * 16;
    yRot = 90 * 16;
    zRot = 0; //-90 * 16;
    matrixProjection[0] = 2 * intr(0, 0) / projSize.width;
    matrixProjection[1] = 0;
    matrixProjection[2] = 1 - (2 * intr(0, 2) / projSize.width);
    matrixProjection[3] = 0;
    matrixProjection[4] = 0;
    matrixProjection[5] = 2 * intr(1, 1) / projSize.height;
    matrixProjection[6] = -1 + ((2 * intr(1, 2) + 2) / projSize.height);
    matrixProjection[7] = 0;
    matrixProjection[8] = 0;
    matrixProjection[9] = 0;
    matrixProjection[10] = (1000 + 0.1) / (1000 - 0.1);
    matrixProjection[11] = (2 * 1000 * 0.1) / (0.1 - 1000);
    matrixProjection[12] = 0;
    matrixProjection[13] = 0;
    matrixProjection[14] = -1;
    matrixProjection[15] = 0;

    pointcloud_.reset(new Cloud);
//    qRegisterMetaType<void> ("void");
    connect(control_, SIGNAL(newCloud()), this, SLOT(updateContent()));
}

OpenGLFullscreen::~OpenGLFullscreen() {
    delete ui;
}

static void qNormalizeAngle(int &angle) {
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

////! [5]
//void OpenGLFullscreen::setXRotation(int angle) {
//    qNormalizeAngle(angle);
//    if (angle != xRot) {
//        xRot = angle;
////        emit xRotationChanged(angle);
//        updateGL();
//    }
//}
////! [5]
//
//void OpenGLFullscreen::setYRotation(int angle) {
//    qNormalizeAngle(angle);
//    if (angle != yRot) {
//        yRot = angle;
////        emit yRotationChanged(angle);
//        updateGL();
//    }
//}
//
//void OpenGLFullscreen::setZRotation(int angle) {
//    qNormalizeAngle(angle);
//    if (angle != zRot) {
//        zRot = angle;
////        emit zRotationChanged(angle);
//        updateGL();
//    }
//}

void OpenGLFullscreen::updateContent() {
//    pointcloud_ = model_->getTransformedModel();
//    std::cout << "FS Cloud: " << pointcloud_->size() << " points" << std::endl;
    paintGL();
    updateGL();
}

void OpenGLFullscreen::initializeGL() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
//    glEnable(GL_LIGHTING);
//    glEnable(GL_LIGHT0);
//    glEnable(GL_COLOR_MATERIAL);
//    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_MULTISAMPLE);
//    static GLfloat lightPosition[4] = { 0.2, 0.2, 0.0, 1.0 };
//    static GLfloat LightAmbient[] = { 0, 0, 0, 1 }; // Ambient Light Values
//    static GLfloat LightDiffuse[] = { 1, 1, 1, 1 }; // Diffuse Light Values
//    static GLfloat LightSpecular[] = { 1, 1, 1, 1 }; // Light Position
//    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
//    glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient); // Setup The Ambient Light
//    glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse); // Setup The Diffuse Light
//    glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
}

void drawDimensions() {

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(30.0f, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 30.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 30.0f);
    glColor3f(1.0f, 1.0f, 1.0f);
    glEnd();

}

void OpenGLFullscreen::paintGL() {
    glPushMatrix();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, 0.0);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);
    if (pointcloud_ != 0) {
        for (unsigned int i = 0; i < pointcloud_->points.size(); i++) {
//            uint32_t rgb = pointcloud_->points[i].rgba;
//            float r = float((rgb >> 16) & 0x0000ff) / 255;
//            float g = float((rgb >> 8) & 0x0000ff) / 255;
//            float b = float((rgb) & 0x0000ff) / 255;
//            glColor3f(r, g, b);
            glVertex3f(pointcloud_->points[i].y, pointcloud_->points[i].z,
                    pointcloud_->points[i].x);
        }
    } else {
        std::cout << ".";
    }
    glEnd();
//    drawDimensions();
    glPopMatrix();
}

const GLfloat modelViewInit[] = { 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0,
        1 };
void OpenGLFullscreen::setCloud(CloudConstPtr cloud) {
//    pointcloud_ = cloud;
    pcl::transformPointCloud(*cloud, *pointcloud_, extr);
}

void OpenGLFullscreen::resizeGL(int width, int height) {
    int side = qMin(width, height);
    if (height == 0)
        height = 1;
    float ratio = (GLfloat) width / (GLfloat) height;
    glViewport(0, 0, (GLint) width, (GLint) height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0f, ratio, 0.1f, 1000.0f);
    glMultMatrixf(matrixProjection);

    glMatrixMode(GL_MODELVIEW);
}

//void OpenGLFullscreen::mousePressEvent(QMouseEvent* event) {
//    lastPos = event->pos();
//}
//
//void OpenGLFullscreen::mouseMoveEvent(QMouseEvent* event) {
//    int dx = event->x() - lastPos.x();
//    int dy = event->y() - lastPos.y();
//
//    if (event->buttons() & Qt::LeftButton) {
//        setXRotation(xRot + 4 * dy);
//        setYRotation(yRot + 4 * dx);
//    } else if (event->buttons() & Qt::RightButton) {
//        setXRotation(xRot + 4 * dy);
//        setZRotation(zRot + 4 * dx);
//    }
//    lastPos = event->pos();
//}

