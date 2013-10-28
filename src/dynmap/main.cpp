/**
 * Needs PCL-1.7-rv8129
 */

#include <pcl/common/eigen.h>
#include "AppController.h"
#include <QApplication>
#include <QtGui/QApplication>


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <sstream>
#include <iomanip>



int main(int argc, char *argv[]) {
    QApplication::setGraphicsSystem("raster");
    QApplication a(argc, argv);

    AppController control;

    int ret = a.exec();

    return (ret);
}
