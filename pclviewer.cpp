#include "pclviewer.h"
#include "common.h"
#include "../build/ui_pclviewer.h"
#include <unistd.h>
#include <QString>
#include <QFileDialog>

// ---------------------------------------------------------------------------
//PCLViewer Constructor
//      Initializes GUI
// ---------------------------------------------------------------------------
PCLViewer::PCLViewer (QWidget *parent) :
            QMainWindow (parent),
            ui (new Ui::PCLViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCL Viewer");

    //initialising tool bar and menu bar icons
    ui->actionPlay->setEnabled(false);
    ui->actionRecord->setEnabled(false);
    ui->actionStop->setEnabled(false);
    ui->actionCsv->setEnabled(false);

    // Set up the QVTK window for visualizing point cloud data from Lidar_1
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0,0,100,0,1,0);
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->addCoordinateSystem(10);
    add_grid(viewer); // add grid to viewer

    ui->qvtkWidget->update ();

    //Set up the QVTK window for visualizing point cloud data from Lidar_2
    viewer2.reset(new pcl::visualization::PCLVisualizer ("viewer2", false));
    ui->qvtkWidget_2->SetRenderWindow (viewer2->getRenderWindow());
    viewer2->setBackgroundColor(0,0,0);
    viewer2->setCameraPosition(0,0,100,0,1,0);
    viewer2->setupInteractor (ui->qvtkWidget_2->GetInteractor(), ui->qvtkWidget_2->GetRenderWindow());
    viewer2->addCoordinateSystem(10);
    add_grid(viewer2);

    ui->qvtkWidget_2->update();

    record =false;  //default value (disabling record at start of GUI)
    offline = false;
    ui->textEdit->setEnabled(false);  //disable textEdit
    //ui->textEdit->append("Status window");

    //add BufferMode CheckBox and buffer Button
    bufferCheckBox = new QCheckBox(this);
    bufferButton = new QPushButton("Buffer", this);
    bufferCheckBox->setEnabled(false);
    bufferButton->setEnabled(false);
    bufferCheckBox->connect(bufferCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onEnableBuffer_stateChanged(int)));
    bufferButton->connect(bufferButton, SIGNAL(pressed()), this, SLOT(onBufferButton_clicked()));
    ui->toolBar->addWidget(bufferCheckBox);
    ui->toolBar->addWidget(bufferButton);
}

// ---------------------------------------------------------------------------
//PCLViewer Destructor
// ---------------------------------------------------------------------------
PCLViewer::~PCLViewer ()
{
    delete ui;
}



void PCLViewer::add_grid(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    pcl::PointXYZRGBA point_1;
    pcl::PointXYZRGBA point_2;

    double grid_color[3] = {1, 1, 1}; //RGB value

    // add x-axis grid marks
    for(int i = -10; i < 11; i++){
        point_1.x = -100;
        point_1.y = i * 10;
        point_1.z = 0;

        point_2.x = 100;
        point_2.y = i * 10;
        point_2.z = 0;

        // two points followed by normalized RGB values and the label
        viewer->addLine<pcl::PointXYZRGBA> (point_1, point_2, grid_color[0], grid_color[1], grid_color[2], "line_x_" + std::to_string(i + 10));
    }

    // add y-axis grid marks
    for(int i = -10; i < 11; i++){
        point_1.y = -100;
        point_1.x = i * 10;
        point_1.z = 0;

        point_2.y = 100;
        point_2.x = i * 10;
        point_2.z = 0;

        viewer->addLine<pcl::PointXYZRGBA> (point_1, point_2, grid_color[0], grid_color[1], grid_color[2], "line_y_" + std::to_string(i + 10));
    }

    // add circular grid marks
    pcl::ModelCoefficients circle_coeff;
    circle_coeff.values.resize (3);    // We need 3 values

    for(int i = 1; i < 11; i++){
        circle_coeff.values[0] = 0;
        circle_coeff.values[1] = 0;
        circle_coeff.values[2] = i * 10;

        viewer->addCircle(circle_coeff, "circle_" + std::to_string(i + 10));
    }

    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "circle_11");

}




// ---------------------------------------------------------------------------
//  void PCLViewer::onUpdate(int Number, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
//
//  Summary:    This function takes the point cloud data pointer from Thread_1
//              and updates the QVTK widget related to Lidar_1
//
//  Parameters:
//
//       cloud  pointer to PointCloud data initialised in Thread_1
//
//  Returns:    nothing
// ---------------------------------------------------------------------------
void PCLViewer::onUpdate(int Number, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    //if the cloud is not yet added to viewer,
    if(!viewer->updatePointCloud(cloud, "cloud"))
    {
        viewer->addPointCloud(cloud, "cloud");
    }

    //Continously changes point size to two
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    ui->qvtkWidget->update ();
}

// ---------------------------------------------------------------------------
//  void PCLViewer::onUpdate2(int Number, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
//
//  Summary:    This function takes the point cloud data pointer from Thread_2
//              and updates the QVTK widget related to Lidar_2
//
//  Parameters:
//
//       cloud  pointer to PointCloud data initialised in Thread_2
//
//  Returns:    nothing
// ---------------------------------------------------------------------------
void PCLViewer::onUpdate2(int Number, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    //mutex.lock();
    if(!viewer2->updatePointCloud(cloud, "cloud"))
    {
        viewer2->addPointCloud(cloud, "cloud");
    }
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    ui->qvtkWidget_2->update();
}

// ---------------------------------------------------------------------------
//  void PCLViewer::setImage(cv::Mat *frame)
//
//  Summary:    This function updates the frame received from camera into
//              OpenCV widget present in this thread(main thread or GUI Thread).
//              Pointer to frame which is initialised in video thread should be
//              deleted only after updating OpenCV widget.
//
//  Parameters:
//
//       frame  pointer to cv::Mat which is initialised in video Thread.
//
//  Returns:    nothing
// ---------------------------------------------------------------------------
void PCLViewer::setImage(cv::Mat *frame)
{
    ui->camWidget->setPixmap(QPixmap::fromImage(cvwidget->image));
    delete frame;
}

// ---------------------------------------------------------------------------
//  void PCLViewer::onBufferButton_clicked()
//
//  Summary:    This function is called only when user clicks bufferButton.
//              It generates timestamped filenames and initializes buffer transmission.
//
// ---------------------------------------------------------------------------
void PCLViewer::onBufferButton_clicked()
{
    QDateTime dateTime = QDateTime::currentDateTime();
    QString fileName = dateTime.toString("yyyyMMdd_hhmmss'_Buffer'");
    mThread->fileName = fileName + "_Lidar_I.pcap";
    mThread2->fileName = fileName + "_Lidar_II.pcap";
    cvwidget->fileName = fileName + "_out.avi";

    cvwidget->startBuffer = true;
    mThread->startBuffer = true;
    mThread2->startBuffer = true;

    //diable record and csv button
    ui->actionRecord->setEnabled(false);
    ui->actionCsv->setEnabled(false);
}

// ---------------------------------------------------------------------------
//  void PCLViewer::bufferModeUpdate()
//
//  Summary:    This function receives the update from all three threads(for saving
//              buffer data structure) after buffer is emptied. Can be used to compress
//              files after buffer is emptied
// ---------------------------------------------------------------------------
void PCLViewer::bufferModeUpdate()
{
    numOfBufSignals++;
    qDebug() << "bufferEmptied signal received" << numOfBufSignals;
    if(((!cvwidget->enableCamera && (numOfBufSignals == 2))||(numOfBufSignals == 3)))
    {
        numOfBufSignals = 0;
        //enable record and csv buttons
        ui->actionRecord->setEnabled(true);
        ui->actionCsv->setEnabled(true);
    }
}

// ---------------------------------------------------------------------------
//  void PCLViewer::onEnableBuffer_stateChanged(int arg1)
//
//  Summary:    This function will enable the bufferButton when checkbox is clicked
//
//  Parameters:
//
//       arg1  contains the status of the checkButton.
//              Qt::Checked - if checkBox is clicked
//              Qt::Unchecked - if disabled
//
// ---------------------------------------------------------------------------
void PCLViewer::onEnableBuffer_stateChanged(int arg1)
{
    if(Qt::Checked == arg1){
        //enable buffer
        bufferButton->setEnabled(true);
        mThread->enableBuffer = true;
        mThread2->enableBuffer = true;
    }
    else if(Qt::Unchecked == arg1){
        //disable buffer
        bufferButton->setEnabled(false);
        mThread->enableBuffer = false;
        mThread2->enableBuffer = false;
    }
}

// ---------------------------------------------------------------------------
// void PCLViewer::on_actionCsv_triggered()
//
//  Summary:
//
//  Parameters:
// ---------------------------------------------------------------------------
void PCLViewer::on_actionCsv_triggered()
{
    //csv button
    csvForm = new csvFrameSelector(this);
    connect(csvForm, SIGNAL(updateCsvForm()), this, SLOT(onCsvFrameUpdate()));
    csvForm->show();
}

// ---------------------------------------------------------------------------
// void PCLViewer::onCsvFrameUpdate()
//
//  Summary:
//
//  Parameters:
// ---------------------------------------------------------------------------
void PCLViewer::onCsvFrameUpdate()
{
    //get user input
    if(csvForm->currentFrame){
        mThread->frameNumber = 1;
        mThread2->frameNumber = 1;
    }
    if(csvForm->allFrames){
        mThread->frameNumber = -1;
        mThread2->frameNumber = -1;
    }
    if(csvForm->framesFrom){
        mThread->frameNumber = csvForm->endFrame;
        mThread2->frameNumber = csvForm->endFrame;
    }

}

// ---------------------------------------------------------------------------
//  void PCLViewer::on_actionOpenCapturedFile_triggered()
//
//  Summary:                        offline mode
//              This function is invoked when "Captured File" icon on ToolBar or MenuBar
//              is clicked. This will create three threads, one for playing video and other two
//              for displaying Lidar data on widgets. A DialogBox
//              is created to obtain the filename form user. After receiving the file name from
//              user, time stamp is extracted from it and used to generate other two flie names.
//              Finally after starting threads, GUI is updating accordingly.
//
// ---------------------------------------------------------------------------
void PCLViewer::on_actionOpenCapturedFile_triggered()
{
    offline = true;

    //initialise Threads
    mThread = new LidarOne(this);
    mThread2 = new LidarTwo(this);

    mThread->pause = false;
    mThread2->pause = false;
    mThread->offline = true;
    mThread2->offline = true;

    connect (mThread, SIGNAL(updateCloud(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)), this, SLOT(onUpdate(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)));
    connect (mThread2, SIGNAL(updateCloud(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)), this, SLOT(onUpdate2(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)));

    //open FileSystemEditor to obtain file from user
    QString filePath, fileName, timeStamp;
    filePath = QFileDialog::getOpenFileName(this,
                                     tr("Open pcap file"), QDir::currentPath());
    qDebug() << filePath;
    if(filePath.isEmpty()){ //return if input is "cancel" from user
        offline = false;
        return;
    }
    else{  //check the extension of the file and extract time stamp
       if((!filePath.endsWith(".pcap")) && (!filePath.endsWith(".avi"))){
           offline = false;
           return;
       }
       fileName = filePath.section('/', -1);    //get the file name from file path
       timeStamp = fileName.section('_', 0, 1);
       if(fileName.section('_', 2,2) == "Buffer"){
           timeStamp = timeStamp + "_Buffer";
       }
       filePath = filePath.section('/', 0, -2) + "/" + timeStamp;
       mThread->fileName = filePath + "_Lidar_I.pcap";
       mThread2->fileName = filePath + "_Lidar_II.pcap";
    }

    //start threads
    mThread->start();
    mThread2->start();

    //camera
    ui->camWidget->setScaledContents(true);
    cvwidget = new video(this);
    cvwidget->fileName = filePath + "_out.avi";
    cvwidget->camera.open(cvwidget->fileName.toStdString().c_str());
    if(!cvwidget->camera.isOpened()){
        qDebug() << "video file not found";
    }
    else{
        cvwidget->liveMode = false;
        this->connect (cvwidget, SIGNAL(imageReady(cv::Mat *)), this, SLOT (setImage(cv::Mat *)));
        cvwidget->start();
    }

    //update GUI
    ui->actionLiveStream->setEnabled(false);
    ui->actionOpenCapturedFile->setEnabled(false);
    ui->actionStop->setEnabled(true);
    ui->actionPlay->setEnabled(true);
    bufferCheckBox->setEnabled(false);
    ui->actionCsv->setEnabled(true);
}

// ---------------------------------------------------------------------------
//  void PCLViewer::on_actionPlay_triggered()
//
//  Summary:    This function is called when "Play/Pause" button is clicked.
//              It will toggle the pause variables in each thread and updates icon.
//              This is only enbled in offline mode.
//
// ---------------------------------------------------------------------------
void PCLViewer::on_actionPlay_triggered()
{
    if(mThread->pause){ //play
        mThread->pause = false;
        mThread2->pause = false;
        cvwidget->pauseVideo = false;
        cvwidget->timer->start(50,cvwidget);
        ui->actionPlay->setIcon(QIcon(":/images/media-playback-pause.png"));
    }
    else{ //pause
        mThread->pause = true;
        mThread2->pause = true;
        cvwidget->pauseVideo = true;
        ui->actionPlay->setIcon(QIcon(":/images/media-playback-start.png")); //update icon accordingly
    }
}

// ---------------------------------------------------------------------------
//  void PCLViewer::on_actionLiveStream_triggered()
//
//  Summary:    Live Mode
//              This function is invoked when user clicks on "Sensor Stream" icon.
//              It creates three threads, one for opening camera and two for displaying data
//              from ethernet ports. Name of the ethernet ports are obtained from user using QDialog.
// ---------------------------------------------------------------------------
//live mode
void PCLViewer::on_actionLiveStream_triggered()
{
    offline = false;

    //initialise Threads
    mThread = new LidarOne(this);
    mThread2 = new LidarTwo(this);

    mThread->pause = false;
    mThread2->pause = false;
    mThread->offline = false;
    mThread2->offline = false;

    connect (mThread, SIGNAL(updateCloud(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)), this, SLOT(onUpdate(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)));
    connect (mThread2, SIGNAL(updateCloud(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)), this, SLOT(onUpdate2(int , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)));

    //Buffer Mode
    numOfBufSignals = 0;
    this->connect(mThread, SIGNAL(bufferEmptied()), this, SLOT(bufferModeUpdate()));
    this->connect(mThread2, SIGNAL(bufferEmptiedII()), this, SLOT(bufferModeUpdate()));

    //Get port numbers from user
    QDialog getEthernetPorts;
    QFormLayout form(&getEthernetPorts);
    form.addRow(new QLabel("Enter Ethernet ports"));
    QLineEdit portOne(&getEthernetPorts), portTwo(&getEthernetPorts);
    form.addRow(tr("&Port 1"), &portOne);
    form.addRow(tr("&Port 2"), &portTwo);
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &getEthernetPorts);
    form.addRow(&buttonBox);
    QObject::connect(&buttonBox, SIGNAL(accepted()), &getEthernetPorts, SLOT(accept()));
    QObject::connect(&buttonBox, SIGNAL(rejected()), &getEthernetPorts, SLOT(reject()));
    portOne.setText("eth14");   //default port names
    portTwo.setText("eth12");
    if(getEthernetPorts.exec() == QDialog::Accepted){
        mThread->portName = portOne.text();
        mThread2->portName = portTwo.text();
    }
    else{
        delete mThread;
        delete mThread2;
        return;
    }

    //Camera
    ui->camWidget->setScaledContents(true);
    cvwidget = new video(this);
    cvwidget->camera.open(0);
    if(!cvwidget->camera.isOpened()){
        qDebug() << "camera is disabled";
        cvwidget->enableCamera = false;
    }
    else{
        cvwidget->enableCamera = true;
        cvwidget->liveMode = true;
        ui->camWidget->setPixmap(QPixmap::fromImage(cvwidget->image));
        this->connect (cvwidget, SIGNAL(imageReady(cv::Mat *)), this, SLOT (setImage(cv::Mat *)));
        this->connect(cvwidget, SIGNAL(bufferEmptied()), this, SLOT(bufferModeUpdate()));
        cvwidget->record = false;
        cvwidget->start();  //start camera
    }

    //start threads
    mThread->start();
    mThread2->start();

    //update GUI
    ui->actionRecord->setEnabled(true);
    bufferCheckBox->setEnabled(true);
    ui->actionLiveStream->setEnabled(false);
    ui->actionOpenCapturedFile->setEnabled(false);
    ui->actionStop->setEnabled(true);
    ui->actionPlay->setEnabled(false);
    ui->actionCsv->setEnabled(true);
}

// ---------------------------------------------------------------------------
//  void PCLViewer::on_actionRecord_triggered()
//
//  Summary:    This will record and save the data into a timestamped files when user
//              clicks "Record" and "Stop Recording" icons. This will create two new threads
//              for saving the data from the ethernet ports. Video thread created in
//              "on_actionLiveStream_triggered" is used for saving the camera data without creating
//              a seperate thread.
// ---------------------------------------------------------------------------
void PCLViewer::on_actionRecord_triggered()
{
    if(!record){ //start recording
        record = true;

        //generate fileName
        QDateTime dateTime = QDateTime::currentDateTime();
        QString timeStamp = dateTime.toString("yyyyMMdd_hhmmss");

        //update GUI
        ui->actionRecord->setEnabled(false); //disable record icon till this function is executed
        ui->actionExit->setEnabled(false);
        ui->actionStop->setEnabled(false);  //disbale stop icon(for switching modes) while recording

        //Instantiate two objects of RecordPcapData
        fromLidarOne = new RecordPcapData(num_bytes_I, this); //for velodyne lidar
        fromLidarTwo = new RecordPcapData(num_bytes_II, this);
        fromLidarOne->record = true;
        fromLidarTwo->record = true;
        fromLidarOne->fileName = timeStamp + "_Lidar_I.pcap";
        fromLidarTwo->fileName = timeStamp + "_Lidar_II.pcap";
        fromLidarOne->portName = mThread->portName;
        fromLidarTwo->portName = mThread2->portName;

        //start threads
        fromLidarOne->start();
        fromLidarTwo->start();
        if(mThread->enableBuffer & mThread2->enableBuffer)
            bufferButton->setEnabled(false); //disable buffer button while recording

        // if camera is available, open videoWriter
        if(cvwidget->enableCamera){
            cvwidget->fileName = timeStamp + "_out.avi";
            int frame_width = cvwidget->camera.get(CV_CAP_PROP_FRAME_WIDTH);
            int frame_height = cvwidget->camera.get(CV_CAP_PROP_FRAME_HEIGHT);
            cvwidget->videoRecord  = new cv::VideoWriter(cvwidget->fileName.toStdString().c_str(),
                                                            CV_FOURCC('M','J', 'P', 'G'), 10,
                                                            cv::Size(frame_width, frame_height), true);
            cvwidget->record = true;
        }

        //update Icon on toolbar
        ui->actionRecord->setEnabled(true);
        ui->actionRecord->setIcon((QIcon(":/images/media-stopRecording.png")));

    }

    else if(record){ //stop recording and save files into a timestamped tar file

        record = false;
        ui->actionRecord->setEnabled(false); //disable stopRecord icon

        fromLidarOne->record = false; fromLidarTwo->record = false;

        //camera
        if(cvwidget->enableCamera){
            cvwidget->record = false;
            delete cvwidget->videoRecord;
        }

        //update GUI
        ui->actionStop->setEnabled(true);
        ui->actionExit->setEnabled(true);
        if(mThread->enableBuffer & mThread2->enableBuffer)
            bufferButton->setEnabled(true);
        ui->actionRecord->setEnabled(true);
        ui->actionRecord->setIcon(QIcon(":/images/media-record.png"));
    }
}

// ---------------------------------------------------------------------------
//  void PCLViewer::on_actionStop_triggered()
//
//  Summary:    Stop icon is used to switch the mode from offline to live and vice-versa
//              This function will clear PCL windows and camera window. This will stop
//              and clean up the memory allocated for Lidar and video thread. And finally it
//              resets GUI.
// ---------------------------------------------------------------------------
void PCLViewer::on_actionStop_triggered()
{
    offline = false;
    mThread->stop = true;
    mThread2->stop = true;
    cvwidget->pauseVideo = true;

    //reset the cvwidget
    cv::Mat *frame = new cv::Mat;
    frame->setTo(cv::Scalar(1,1,1));
    const QImage dummy(frame->data, frame->cols, frame->rows, frame->step, QImage::Format_RGB888);
    ui->camWidget->setPixmap(QPixmap::fromImage(dummy));

    //wait for threads to exit eventloop
    if(!mThread->wait(5000)){
        mThread->terminate();
        mThread->wait();
    }

    if(!mThread2->wait(5000)){
        mThread2->terminate();
        mThread2->wait();
    }

    if(!cvwidget->wait(5000)){
        cvwidget->terminate();
        cvwidget->wait();
    }
    qDebug() << "simulation stopped";

    delete cvwidget;
    delete mThread;
    delete mThread2;
    delete frame;

    //reset GUI to initial state
    ui->actionLiveStream->setEnabled(true);
    ui->actionOpenCapturedFile->setEnabled(true);
    ui->actionStop->setEnabled(false);
    ui->actionPlay->setEnabled(false);
    ui->actionRecord->setEnabled(false);
    ui->actionCsv->setEnabled(false);
    ui->actionPlay->setIcon(QIcon(":/images/media-playback-pause.png"));
    ui->actionRecord->setIcon(QIcon(":/images/media-record.png"));
}

// ---------------------------------------------------------------------------
//  void PCLViewer::on_actionExit_triggered()
//
//  Summary:    This is a invoked when close button is clicked. This will call the
//              close() function of the main window which in raises a event to closeEvent() event handler.
// ---------------------------------------------------------------------------
void PCLViewer::on_actionExit_triggered()
{
    this->close();
}

// ---------------------------------------------------------------------------
//  void PCLViewer::closeEvent(QCloseEvent *event)
//
//  Summary:    This is event Handler which handles the close request from the widget.
//              It can be used to ask the user for confirmation of closing the window.
// ---------------------------------------------------------------------------
void PCLViewer::closeEvent(QCloseEvent *event)
{
    event->accept();
}

void PCLViewer::on_actionAbout_triggered()
{
    QMessageBox::information(this, tr("PCLViewer"), tr("Version 1.0"));
}
