#include "vision.h"
#include "ui_vision.h"
#include "detect.h"
#include "calib.h"
#include "define_parameter.h"



Vision::Vision(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Vision)
{
    ui->setupUi(this);
    VideoCapture = new detect(this);
    CalibFrame = new Calib(this);
    connect(VideoCapture, &detect::newPixmapCaptured, this, [&]()
    {
       if(check_Closing){
           check_Closing = false;
           VideoCapture->terminate();
           VideoCapture->wait();
           VideoCapture->quit();
           VideoCapture->wait();
       } else{
           VideoCapture->Check_accept_opened =true;             //Accept for continuous video capture
           ui->CameraFrame->setPixmap(VideoCapture->pixmap().scaled(640,480));
           CalibFrame -> buffer.clear();
           CalibFrame -> buffer = VideoCapture ->buffer ;
           CalibFrame -> Set = true;
       }
    });
    connect(CalibFrame, &Calib::newPixmapCaptured, this, [&]()
    {
        if( enable == true)
        {
            for (size_t i = 0; i < CalibFrame -> Send_buffer.size(); ++i)
            {

                count_object ++;
                qDebug()<<"Toa do da gui cua vat" << count_object << ": X = "<< CalibFrame-> Send_buffer[i][1]<< " Y = "<< CalibFrame-> Send_buffer[i][2]<<
                            "ID ="<< (ObjectType)CalibFrame->Send_buffer[i][0] << "Theta = " << CalibFrame->Send_buffer[i][4] <<endl;
                if (ui->checkBox_2->isChecked()== true)
                {
                    send_packet(CalibFrame->Send_buffer[i][1], CalibFrame->Send_buffer[i][2], CalibFrame->Send_buffer[i][4],
                                                (ObjectType)CalibFrame->Send_buffer[i][0] ,mSerial);

                }
            }
        }
        CalibFrame-> Send_buffer.clear();

    });
}

Vision::~Vision()
{
    VideoCapture -> mVideoCap.release();
    delete CalibFrame;
    delete VideoCapture;
    delete ui;
}
void Vision::closeEvent (QCloseEvent *event)
{
    if (check_Opened) {
        event->ignore();
    } else {
        event->accept();
    }
}

void Vision::send_packet(double x, double y, double roll, ObjectType flag_type, QSerialPort* mSerial)
{
    QByteArray command;
    command.append(0x28);
    command.append("00");
    command.append(0x01);
    command.append(24);
    ADD_VALUE(&command, x, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, y, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, roll, SCARA_COR_VALUE_DOUBLE);
    command.append(flag_type);
    command.append("})");
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void Vision::on_CameraOn_Button_clicked()
{
    VideoCapture->start();
    CalibFrame->start();
    check_Opened = true;     // Check for close button on apllication
    ui->CameraOn_Button->setEnabled(false);
    VideoCapture->Check_accept_opened = true; //Accept for continuous video capture
}

void Vision::on_CameraOff_Button_clicked()
{
    CalibFrame -> terminate();
    CalibFrame -> wait();
    CalibFrame ->quit();
    CalibFrame -> wait();

    check_Opened = false;
    check_Closing = true;
    ui->CameraOn_Button->setEnabled(true);

}



void Vision::on_checkBox_stateChanged(int arg1)
{
    if(ui->checkBox->isChecked() == true){
        enable = true;
    }else{
        enable = false;
    }
}


void Vision::on_ResetPoint_clicked()
{
    CalibFrame->current_point = 0;
}
