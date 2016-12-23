#include "csvframeselector.h"
#include "../build/ui_csvframeselector.h"

csvFrameSelector::csvFrameSelector(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::csvFrameSelector)
{
    ui->setupUi(this);
    currentFrame = false;
    allFrames = false;
    framesFrom = false;
    endFrame = 0;
}

csvFrameSelector::~csvFrameSelector()
{
    delete ui;
}

void csvFrameSelector::on_buttonBox_accepted()
{
    if(ui->currentFrame->isChecked())
        currentFrame = true;
    if(ui->allFrames->isChecked())
        allFrames = true;
    if(ui->framesFrom->isChecked()){
        framesFrom = true;
        endFrame = ui->endFrameNumber->value();
    }
    emit updateCsvForm();
}
