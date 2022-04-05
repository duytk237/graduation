#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "receivethread.h"
#include "define_parameter.h"
#include "packet_handler.h"
#include "gcode_decoder.h"
#include "vision.h"

#define SET_GCODE_UI(value){         \
    ui->label_59->setEnabled(value);         \
    ui->tb_gcode_wc->setEnabled(value);   \
}
#define METHOD_TAB_ENABLE(index, state) {                    \
    if(state == true){                                       \
        ui->tw_method_content->setEnabled(true);             \
        for(int i = 0; i < 5; i++){                          \
            if(i == index){                                  \
              ui->tw_method_content->setTabEnabled(i, true); \
            }else{                                           \
              ui->tw_method_content->setTabEnabled(i, false);\
            }                                                \
        }                                                    \
        ui->tw_method_content->setCurrentIndex(index);       \
    }else{                                                   \
        ui->tw_method_content->setEnabled(false);            \
    }                                                        \
}
#define SET_CONTROL_UI_STATE(state) {        \
    ui->tw_method_content->setEnabled(state);\
    ui->gb_position_read->setEnabled(state); \
    ui->gb_method_change->setEnabled(state); \
    ui->gb_output_shaft->setEnabled(state);  \
}
#define CHOOSE_CENTER1 { \
    centerX = center_x1; \
    centerY = center_y1; \
}
#define CHOOSE_CENTER2 { \
    centerX = center_x2; \
    centerY = center_y2; \
}

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    ReceiveThread *Received_Thread;
    packet_handler *_packet_handler = new packet_handler();
    std::vector<uint8_t> FIFO_Buffer;


private slots:
    void on_bt_refresh_clicked();

    void on_bt_connect_clicked();

    void on_bt_robot_stop_clicked();
    void on_bt_scan_limit_clicked();

    void on_bt_home_clicked();

    void on_bt_movL_clicked();

    void on_bt_model_setting_clicked();

    void on_bt_on_magnet_clicked();

    void on_bt_off_magnet_clicked();

    void on_bt_read_position_clicked();

    void on_bt_keycommand();

    void on_bt_key_setsp_clicked();

    void on_bt_set_method_clicked();

    void on_bt_clear_console_clicked();

    void on_testing_clicked();

    void on_bt_testmt();

    void on_bt_conveyor_sp_clicked();

    void on_bt_browse_clicked();

    void on_bt_process_clicked();

    void on_bt_gcode_start_clicked();

    void on_bt_gcode_stop_clicked();

    void on_bt_gcode_pause_clicked();

    void on_bt_gcode_resume_clicked();

    void on_bt_gcode_configure_clicked();

    void on_bt_movC1_clicked();

    void on_bt_movC2_clicked();

    void on_bt_sw_test_clicked();

    void on_bt_conveyor_refresh_clicked();

    void on_bt_conveyor_start_clicked();

    void on_bt_conveyor_stop_clicked();

    void on_bt_conveyor_configure_clicked();

    void on_bt_conveyor_reverse_clicked();

    void on_bt_conveyor_con_clicked();

    void on_tb_conveyor_pulse_textChanged(const QString &arg1);

    void on_rb_gcode_lspb_clicked();

    void on_rb_gcode_linear_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_bt_load_offset_clicked();

    void on_bt_save_browse_clicked();

    void on_bt_save_data_file_clicked();

public slots:
    void received_callback(QByteArray log_data);
    void display_event(Display_packet data);

private:
    bool save_enable = false;
    QString DAQ_content = "X,Y,Z,Roll,Theta1,Theta2,D3,Theta4\r";
    Ui::MainWindow *ui;
    void object_detected(double x, double y, double roll);
    Coordinate_Receive_Handler_TypeDef MovC_ACK = DISPLAY_ONLY;
    void MovC_Hanlder(Coordinate_Receive_Handler_TypeDef type, Display_packet data);
    void DAQ(Display_packet &data);
    Vision *vision;
protected:
    void closeEvent(QCloseEvent *event);
};

#endif // MAINWINDOW_H
