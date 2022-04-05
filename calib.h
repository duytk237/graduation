#ifndef CALIB_H
#define CALIB_H

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <QTime>
#include <QThread>
#include <QPixmap>
#include <QDebug>

#define ID_CAMERA 0

class Calib: public QThread
{
    Q_OBJECT
public:
    Calib(QObject *parent = nullptr);

    std::vector<std::vector<double>> buffer;
    std::vector<std::vector<double>> Trans_buffer;
    std::vector<std::vector<double>> Send_buffer;
    bool Set;
    double precTick, ticks, dT = 0 ;
    size_t current_point = 0;
    int count_delay = 0, count_erase = 0;
    bool Ready;
    double pre_X, pre_id;


signals:
    void newPixmapCaptured();
protected:
    void run() override;
private:
    QTime m_time;


};

#endif // CALIB_H
