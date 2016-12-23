#include "video.h"

// ---------------------------------------------------------------------------
//  video constructor
// ---------------------------------------------------------------------------
video::video(QObject *parent) :
    QThread(parent)
{
    QImage dummy(100, 100, QImage::Format_RGB32);
    image = dummy;
    for (int x=0; x<100; x++){
        for(int y=0; y<100; y++){
            image.setPixel(x, y, qRgb(x, y, y));
        }
    }

    pauseVideo = false;
    record = false;
    startBuffer = false;
}

video::~video()
{

}

// ---------------------------------------------------------------------------
//  void video :: run()
//
//  Summary:    This function will start the timer with interval 50ms. After every 50ms
//              a timerEvent is raised.
// ---------------------------------------------------------------------------
void video::run()
{
    timer = new QBasicTimer;
    timer->start(50,this);
}

// ---------------------------------------------------------------------------
// void video::timerEvent(QTimerEvent *)
//
//  Summary:    This function will get a frame from camera and convert into a QImage
//              and transmit it to main thread to update the cvwidget (camera window).
// ---------------------------------------------------------------------------
void video::timerEvent(QTimerEvent *)
{
    cv::Mat *frame;
    frame = new cv::Mat;
    camera >> *frame;

    videoBuffer.enqueue(*frame);
    if(videoBuffer.size() > num_frame_buffer_video)
        videoBuffer.dequeue();

    if(!frame->data){
        timer->stop();   //stop the timer if there is no data from device
        return;
    }

    if(pauseVideo){     //to pause the video in offline mode
        timer->stop();
    }

    if(record)      //if recording is enabled in live mode, write frame into a file
        videoRecord->write(*frame);

    if(startBuffer){    //save the buffer into a file and emit signal after finished
        startBuffer = false;
        int frame_width = camera.get(CV_CAP_PROP_FRAME_WIDTH);
        int frame_height = camera.get(CV_CAP_PROP_FRAME_HEIGHT);
        cv::VideoWriter bufferVideoFile(fileName.toStdString().c_str(),
                                        CV_FOURCC('M','J', 'P', 'G'),
                                        10, cv::Size(frame_width, frame_height), true);
        while(!videoBuffer.isEmpty()){
            bufferVideoFile.write(videoBuffer.head());
            videoBuffer.dequeue();
        }
        emit bufferEmptied();
    }

    cv::resize(*frame, *frame, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    cv::cvtColor(*frame, *frame, CV_BGR2RGB);
    const QImage dummy(frame->data, frame->cols, frame->rows, frame->step, QImage::Format_RGB888);
    Q_ASSERT(dummy.constBits() == frame->data);
    image = dummy;
    emit imageReady(frame);
}

