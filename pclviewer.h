#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>
#include <string>

// Qt
#include <QMainWindow>
#include <QProcess>
#include <QDateTime>
#include <QVBoxLayout>
#include <QImage>
#include <QMessageBox>
#include <QCheckBox>
#include <QDialog>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QPushButton>
#include <QCloseEvent>

#include "lidarone.h"
#include "lidartwo.h"
#include "recordpcapdata.h"
#include "video.h"
#include "csvframeselector.h"

namespace Ui
{
  class PCLViewer;
}

// ---------------------------------------------------------------------------
// class PCLViewer
// ---------------------------------------------------------------------------
class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
    explicit PCLViewer (QWidget *parent = 0); // constructor
    ~PCLViewer ();  // destructor
    LidarOne *mThread;
    LidarTwo *mThread2;
    RecordPcapData *fromLidarOne;
    RecordPcapData *fromLidarTwo;

    int numOfBufSignals;    //used in Buffer Mode to synchronize threads
    bool record;            //true - if recording pcap data is in process
    bool offline;           //to keep track which mode is currently active

public slots:
    void onUpdate(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
    void onUpdate2(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
    void setImage(cv::Mat *);
    void bufferModeUpdate();

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

    void closeEvent(QCloseEvent *);     //overload closeEvent (can be used if compression of files is done)

private slots:
    void onBufferButton_clicked();
    void onEnableBuffer_stateChanged(int arg1);
    void on_actionExit_triggered();
    void on_actionStop_triggered();
    void on_actionOpenCapturedFile_triggered();
    void on_actionLiveStream_triggered();
    void on_actionPlay_triggered();
    void on_actionRecord_triggered();
    void on_actionCsv_triggered();
    void onCsvFrameUpdate();
    void on_actionAbout_triggered();
    void add_grid(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

private:
   Ui::PCLViewer *ui;

   //camera widget (opencv)
   video *cvwidget;

   //buffer Mode checkBox and pushButton
   QCheckBox *bufferCheckBox;
   QPushButton *bufferButton;
   csvFrameSelector *csvForm;

};

#endif // PCLVIEWER_H
