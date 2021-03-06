cmake_minimum_required (VERSION 3.0 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project      (pcl-visualizer)
find_package (Qt4 REQUIRED)
find_package (VTK 5.8 REQUIRED)
find_package (PCL 1.7.1 REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_library (TINS_LIBRARY tins)

#added pcap lib
include_directories (${PCL_INCLUDE_DIRS} "/use/include/pcap/")
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})
include_directories( ${OpenCV_INCLUDE_DIRS})

set  (project_SOURCES main.cpp pclviewer.cpp lidarone.cpp lidartwo.cpp common.cpp recordpcapdata.cpp video.cpp savebuffer.cpp csvframeselector.cpp)
set  (project_HEADERS pclviewer.h lidarone.h lidartwo.h recordpcapdata.h video.h savebuffer.h csvframeselector.h)
set  (project_FORMS   pclviewer.ui csvframeselector.ui)
set  (project_RESOURCES images.qrc)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})
QT4_ADD_RESOURCES  (project_RESOURCES_RCC ${project_RESOURCES})


INCLUDE (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})


include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE  (pcl_visualizer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC}
                                ${project_RESOURCES_RCC})

TARGET_LINK_LIBRARIES (pcl_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${TINS_LIBRARY} "pcap")

