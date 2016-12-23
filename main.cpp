#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>
//#include <cstdio>
//#include <unistd.h>
//#include <wait.h>
#include <QMetaType>

int main (int argc, char *argv[])
{
      QApplication a (argc, argv);
      qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr >("pcl::PointCloud<pcl::PointXYZRGBA>::Ptr");
      PCLViewer *w = new PCLViewer();

      w->show();

      a.exec ();
      return 0;
}
