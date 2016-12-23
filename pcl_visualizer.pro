QT       += core gui network
QT       += multimedia multimediawidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pcl_visualizer
TEMPLATE = app

QMAKE_CXXFLAGS = -Wno-deprecated \
                 -Wno-unused-variable
QMAKE_CFLAGS = -Wno-unused-variable

CONFIG += link_pkgconfig
PKGCONFIG += opencv

SOURCES += main.cpp\
        pclviewer.cpp \
    common.cpp \
    lidarone.cpp \
    lidartwo.cpp \
    recordpcapdata.cpp \
    video.cpp \
    savebuffer.cpp \
    CMakeLists.txt \
    csvframeselector.cpp


HEADERS  += pclviewer.h \
    common.h \
    lidarone.h \
    lidartwo.h \
    recordpcapdata.h \
    video.h \
    savebuffer.h \
    csvframeselector.h


FORMS    += pclviewer.ui \
    csvframeselector.ui


INCLUDEPATH += /usr/include/pcl-1.7 \
               /usr/include/eigen3 \
               /usr/include/vtk-5.8/

INCLUDEPATH += /usr/include/pcap/

LIBS += -lboost_system \
        -lboost_filesystem \
        -lboost_thread \
        -lboost_date_time \
        -lboost_iostreams \
        -lboost_serialization \
        -lboost_chrono

LIBS += -lpcap

LIBS += -lpcl_common \
        -lpcl_kdtree \
        -lpcl_octree \
        -lpcl_search \
        -lpcl_surface \
        -lpcl_sample_consensus \
        -lpcl_io \
        -lpcl_filters \
        -lpcl_features \
        -lpcl_keypoints \
        -lpcl_registration \
        -lpcl_segmentation \
        -lpcl_recognition \
        -lpcl_visualization \
        -lpcl_people \
        -lpcl_outofcore \
        -lpcl_tracking \
        -lpcl_apps \
	-ltins \

LIBS += /usr/lib/libvtkViews.so.5.8.0 \
        /usr/lib/libvtkInfovis.so.5.8.0 \
        /usr/lib/libvtkWidgets.so.5.8.0 \
        /usr/lib/libvtkVolumeRendering.so.5.8.0 \
        /usr/lib/libvtkHybrid.so.5.8.0 /usr/lib/libvtkParallel.so.5.8.0 /usr/lib/libvtkRendering.so.5.8.0 /usr/lib/libvtkImaging.so.5.8.0 \
        /usr/lib/libvtkGraphics.so.5.8.0 /usr/lib/libvtkIO.so.5.8.0 /usr/lib/libvtkFiltering.so.5.8.0 \
        /usr/lib/libvtkCommon.so.5.8.0 -lm /usr/lib/libvtksys.so.5.8.0 -ldl /usr/lib/libvtkQtChart.so.5.8.0 -Wl,-Bstatic -lflann_cpp_s -Wl,-Bdynamic /usr/lib/libvtkGenericFiltering.so.5.8.0 \
        /usr/lib/libvtkGeovis.so.5.8.0 /usr/lib/libvtkCharts.so.5.8.0 /usr/lib/libQVTK.so.5.8.0

RESOURCES += \
    images.qrc
