QT4_WRAP_CPP(moc_sources
  gui/mainwindow.h
#  gui/PclFullscreen.h
  AppController.h
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
)

QT4_WRAP_UI(uic_sources
  gui/mainwindow.ui
#  gui/PclFullscreen.ui
)

QT4_WRAP_CPP(moc_sources1
  gui/OpenGLFullscreen.h
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
)

QT4_WRAP_UI(uic_sources1
  gui/OpenGLFullscreen.ui
)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

SET(dynmap_sources ${moc_sources} ${uic_sources} ${moc_sources1} ${uic_sources1}
               main.cpp
               gui/mainwindow.cpp                  gui/mainwindow.h
               gui/OpenGLFullscreen.cpp            gui/OpenGLFullscreen.h
               #gui/PclFullscreen.cpp               gui/PclFullscreen.h
               AppController.cpp                   AppController.h
               AppModel.cpp                        AppModel.h
               AppModelData.cpp                    AppModelData.h
               filter/PclPassThroughFilter.cpp     filter/PclPassThroughFilter.h
               filter/PclVoxelGridFilter.cpp       filter/PclVoxelGridFilter.h
               filter/PclPlaneFilter.cpp           filter/PclPlaneFilter.h
               filter/PickedClusterFilter.cpp      filter/PickedClusterFilter.h
               filter/ModelAcquisitionPipeline.cpp filter/ModelAcquisitionPipeline.h
               filter/TrackingPipeline.cpp         filter/TrackingPipeline.h
               grabber/PclOpenNIGrabber.cpp        grabber/PclOpenNIGrabber.h
               helper/ScopeTimeWatch.cpp           helper/ScopeTimeWatch.h        
               calibration/RGBDemoCalibration.cpp  calibration/RGBDemoCalibration.h       
)

NESTK_ADD_EXECUTABLE(dynmap ${dynmap_sources})

TARGET_LINK_LIBRARIES(dynmap nestk QVTK vtkHybrid ${PCL_LIBRARIES})
