#ifndef CSVFRAMESELECTOR_H
#define CSVFRAMESELECTOR_H

#include <QDialog>

namespace Ui {
class csvFrameSelector;
}

class csvFrameSelector : public QDialog
{
    Q_OBJECT

public:
    explicit csvFrameSelector(QWidget *parent = 0);
    ~csvFrameSelector();
    bool currentFrame;      //used to store the user input from csv form
    bool allFrames;
    bool framesFrom;
    int endFrame;          //to store number of frames to record
signals:
    void updateCsvForm();
private slots:
    void on_buttonBox_accepted();

private:
    Ui::csvFrameSelector *ui;
};

#endif // CSVFRAMESELECTOR_H
