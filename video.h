#ifndef VIDEO_H
#define VIDEO_H

#include <QObject>
#include <QThread>
#include <QImage>
#include <QBasicTimer>
#include <QDebug>
#include <QQueue>
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>

const int num_frame_buffer_video = 200; // number of frames per video buffer

// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
class video : public QThread
{
    Q_OBJECT
public:
    video(QObject *parent = 0);
    ~video();
    void run();
    QBasicTimer *timer;
    QImage image;
    cv::VideoCapture camera;
    cv::VideoWriter *videoRecord;
    bool record;
    bool enableCamera;
    bool liveMode;
    bool pauseVideo;
    bool startBuffer;
    QString fileName;
signals:
    void imageReady(cv::Mat *);
    void bufferEmptied();
protected:
    //timer Event for receiving a frame from camera
    void timerEvent(QTimerEvent *);
    QQueue<cv::Mat> videoBuffer;
private:

};

#endif // VIDEO_H
