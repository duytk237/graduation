#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qstring.h>
#include <QDebug>
#include <QtSerialPort/QtSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QFileDialog>
#include <QFile>

QSerialPort *mSerial1;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->bt_connect->setStyleSheet("background-color:red");
    ui->bt_connect->setText("Connect");
    connect(_packet_handler, SIGNAL(on_display_event(Display_packet)), this, SLOT(display_event(Display_packet)));
    QString temp = "bt_inc_";
    for(int i = 1; i <= 8; i++){
        QPushButton *object_bt = this->findChild<QPushButton*>(temp + QString::number(i));
        connect(object_bt, SIGNAL(clicked()), this, SLOT(on_bt_keycommand()));
    }
    connect(ui->bt_start_test, SIGNAL(clicked()), this, SLOT(on_bt_testmt()));
    connect(ui->bt_stop_test, SIGNAL(clicked()), this, SLOT(on_bt_testmt()));
    SET_CONTROL_UI_STATE(false);
     SET_GCODE_UI(false);
     global_ui = ui;
     system_parameter->Load_Configuration();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::display_event(Display_packet data)
{

    switch (data.action_id) {
        case DISPLAY_POSITION:{
            if(MovC_ACK == DISPLAY_ONLY){
                ui->tb_x_cur_cor->setText(QString::number(data.RealData.x));
                ui->tb_y_cur_cor->setText(QString::number(data.RealData.y));
                ui->tb_z_cur_cor->setText(QString::number(data.RealData.z));
                ui->tb_roll_cur_cor->setText(QString::number(data.RealData.roll*180.0f/3.141592f));
                ui->tb_theta1_cur_val->setText(QString::number(data.RealData.theta1));
                ui->tb_theta2_cur_val->setText(QString::number(data.RealData.theta2));
                ui->tb_theta4_cur_val->setText(QString::number(data.RealData.theta4));
                ui->tb_d3_cur_val->setText(QString::number(data.RealData.D3));
//                log_console("Position data received");
//                log_console("--------------------------");
            }else{
                MovC_Hanlder(MovC_ACK, data);
                MovC_ACK = DISPLAY_ONLY;
            }
            if(save_enable == true && ui->cb_save_enable->isChecked() == true){
                DAQ(data);
            }
        }
        break;
        case DISPLAY_RPD_DETAIL:{
            if(_packet_handler->number_of_packet != 0){
                _packet_handler->number_of_packet--;
            }
            QString detail_string;
            for(int t = 0; t < data.Reference_String.length(); t++){
                switch (data.Reference_String.at(t)) {
                case MANUAL_METHOD         :{
                    METHOD_TAB_ENABLE(1, true);
                    detail_string += system_parameter->DETAIL_STATUS[data.Reference_String.at(t)] + "; ";
                }
                break;
                case SEMI_AUTO_METHOD      :{
                    METHOD_TAB_ENABLE(0, true);
                    detail_string += system_parameter->DETAIL_STATUS[data.Reference_String.at(t)] + "; ";
                }
                break;
                case GCODE_METHOD           :{
                    METHOD_TAB_ENABLE(3, true);
                    detail_string += system_parameter->DETAIL_STATUS[data.Reference_String.at(t)] + "; ";
                }
                break;
                case TEST_METHOD           :{
                    METHOD_TAB_ENABLE(2, true);
                    detail_string += system_parameter->DETAIL_STATUS[data.Reference_String.at(t)] + "; ";
                }
                break;
                case PICK_AND_PLACE_METHOD :{
                    METHOD_TAB_ENABLE(4, true);
                    detail_string += system_parameter->DETAIL_STATUS[data.Reference_String.at(t)] + "; ";
                }
                break;
                case MANUAL_SPEED:{
                    detail_string += system_parameter->DETAIL_STATUS[data.Reference_String.at(t)] +
                            ": " + QString::number(data.Reference_String.at(t+1)) + "; ";
                    t++;
                }
                break;
                case OBJECT_DETECTED:{
                    detail_string += system_parameter->DETAIL_STATUS[data.Reference_String.at(t)] +
                            ": " + system_parameter->OBJECT_String[data.Reference_String.at(t+1)] + "; ";
                    t++;
                }
                break;
                default:{
                    detail_string += system_parameter->DETAIL_STATUS[data.Reference_String.at(t)] + "; ";
                }
                break;
                }
            }
            if(data.Respond_Type == RPD_START){
                save_enable = true;
            }else if(data.Respond_Type == RPD_DONE){
                save_enable = false;
            }
            log_console(system_parameter->RDP_String[data.Respond_Type]
                    + " | Command ID: " + system_parameter->COMMAND_STRING[data.Command_ID]
                    + " | Detail: " + detail_string);
            if(_packet_handler->number_of_packet == 0){
                log_console("--------------------------");
            }
        }
        break;
        case DISPLAY_GCODE_PROCESS:{
            ui->pb_gcode_process->setValue(data.Contain_Data.at(0));
        }
        break;
    }

}

void MainWindow::on_bt_refresh_clicked()
{
    ui->com_list->clear();
    const auto list_of_port = QSerialPortInfo::availablePorts();
        for (const QSerialPortInfo &port : list_of_port)
            ui->com_list->addItem(port.portName());
}

void MainWindow::on_bt_connect_clicked()
{
    if (ui->bt_connect->text() == "Connect"){
        ui->bt_connect->setText("Disconnect");
        ui->bt_connect->setStyleSheet("background-color:green");
        mSerial = new QSerialPort(this);
        mSerial->setPortName(ui->com_list->currentText());
        mSerial->setBaudRate(QSerialPort::Baud115200);
        mSerial->setDataBits(QSerialPort::Data8);
        mSerial->setParity(QSerialPort::NoParity);
        mSerial->setStopBits(QSerialPort::OneStop);
        mSerial->setFlowControl(QSerialPort::NoFlowControl);
        mSerial->open(QIODevice::ReadWrite);
        Received_Thread = new ReceiveThread(this);
        Received_Thread->set_serial_object(mSerial);
        connect(Received_Thread, SIGNAL(packet_received(QByteArray)), this, SLOT(received_callback(QByteArray)));
        Received_Thread->start();
        SET_CONTROL_UI_STATE(true);
        METHOD_TAB_ENABLE(0, false);
    }else if(ui->bt_connect->text() == "Disconnect"){
        ui->bt_connect->setText("Connect");
        ui->bt_connect->setStyleSheet("background-color:red");
        Received_Thread->stop = true;
        mSerial->close();
        delete mSerial;
        delete Received_Thread;
        SET_CONTROL_UI_STATE(false);
    }
}

void MainWindow::received_callback(QByteArray log_data)
{
    //qDebug() << log_data;
    FIFO_Buffer.insert(FIFO_Buffer.end(), log_data.begin(), log_data.end());
    _packet_handler->categorize(FIFO_Buffer);
}

void MainWindow::on_bt_robot_stop_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_STOPNOW);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}


void MainWindow::on_bt_scan_limit_clicked()
{
    QByteArray command;
        qDebug() << 1<< command.toStdString().c_str();
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_SCAN_LIMIT);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
                                                qDebug() << "scan: " << command.toHex();
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_home_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_MOVE_LINE);
    ADD_VALUE(&command, 200.0, SCARA_COR_VALUE_TEXT);
    ADD_VALUE(&command, -75.0, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, 133.291, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, 0.0, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, 0.0, SCARA_COR_VALUE_DOUBLE);
    command.append(DUTY_MODE_INIT_QVT);
    ADD_VALUE(&command, 2.0, SCARA_COR_VALUE_DOUBLE);
    command.append(DUTY_COORDINATES_ABS);
    command.append(DUTY_TRAJECTORY_LSPB);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_movL_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    qDebug() << command << command.size();
    command.append(COMMAND_TRANSMISION);
    qDebug() << command << command.size();
    if(ui->rb_movL->isChecked() == true){
        command.append(CMD_MOVE_LINE);
    }else if(ui->rb_movJ->isChecked() == true){
        command.append(CMD_MOVE_JOINT);
    }
    qDebug() << command << command.size();
    ADD_VALUE(&command, ui->tb_x_cor->text(), SCARA_COR_VALUE_TEXT);
    qDebug() << command << command.size();
    ADD_VALUE(&command, ui->tb_y_cor->text(), SCARA_COR_VALUE_TEXT);
    ADD_VALUE(&command, ui->tb_z_cor->text(), SCARA_COR_VALUE_TEXT);
    ADD_VALUE(&command, ui->tb_roll_ang->text(), SCARA_COR_VALUE_TEXT);
    if(ui->tb_v_factor->text().toDouble() > 1 || ui->tb_v_factor->text().toDouble() < 0){
        log_console("Invalid for velocity factor");
        return;
    }else{
        ADD_VALUE(&command, ui->tb_v_factor->text(), SCARA_COR_VALUE_TEXT);
    }

    if(ui->rb_qva->isChecked() == true) {
        command.append(DUTY_MODE_INIT_QVA);
        ADD_VALUE(&command, ui->tb_a_factor->text(), SCARA_COR_VALUE_TEXT);
    }else if(ui->rb_qvt->isChecked() == true){
        command.append(DUTY_MODE_INIT_QVT);
        ADD_VALUE(&command, ui->tb_time->text(), SCARA_COR_VALUE_TEXT);
    }else if(ui->rb_qt->isChecked() == true){
        command.append(DUTY_MODE_INIT_QT);
        ADD_VALUE(&command, ui->tb_time->text(), SCARA_COR_VALUE_TEXT);
    }
    if(ui->rb_abs->isChecked() == true){
       command.append(DUTY_COORDINATES_ABS);
    }else if(ui->rb_inc->isChecked() == true){
       command.append(DUTY_COORDINATES_REL);
    }
    if(ui->rb_lspb->isChecked() == true){
       command.append(DUTY_TRAJECTORY_LSPB);
    }else if(ui->rb_scur->isChecked() == true){
       command.append(DUTY_TRAJECTORY_SCURVE);
    }else if(ui->rb_linear->isChecked() == true){
        command.append(DUTY_TRAJECTORY_LINEAR);
    }
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    qDebug() << command << command.length();
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_model_setting_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_TEST_METHOD_SETTING);
    command.append(ui->tb_m1_pulse->text().toUInt());
    command.append(ui->tb_m2_pulse->text().toUInt());
    command.append(ui->tb_m3_pulse->text().toUInt());
    command.append(ui->tb_m4_pulse->text().toUInt());
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_on_magnet_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_OUTPUT);
    command.append('\1');
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_off_magnet_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_OUTPUT);
    command.append('\0');
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_read_position_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_READ_POSITION);
    if(ui->rb_real_type->isChecked() == true){
        command.append(READ_CONTINUOUS_ENABLE);
    }else if(ui->rb_estimate_type->isChecked() == true){
        command.append(READ_CONTINUOUS_DISABLE);
    }else if(ui->rb_real_data_update->isChecked() == true){
        command.append(POSITION_UPDATE);
    }
    command.append((uint8_t)(ui->tb_update_cycle->text().toUInt()));
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
    MovC_ACK = DISPLAY_ONLY;
}

void MainWindow::on_bt_keycommand()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_KEYBOARD);
    QPushButton *obj_sender = (QPushButton*)sender();
    uint8_t selection = (uint8_t)obj_sender->objectName().split('_')[2].toInt() - 1;
    command.append(selection);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_key_setsp_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_KEY_SPEED);
    uint8_t key_speed = (uint8_t)ui->tb_key_setsp->text().toInt();
    key_speed = (key_speed > SHIFT_KEY_MAX)?SHIFT_KEY_MAX:key_speed;
    key_speed = (key_speed < SHIFT_KEY_MIN)?SHIFT_KEY_MIN:key_speed;
    command.append(key_speed);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_set_method_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_METHOD_CHANGE);
    if(ui->rb_manual->isChecked() == true){
        command.append(SCARA_METHOD_MANUAL);
    }else if(ui->rb_semi_auto->isChecked() == true){
        command.append(SCARA_METHOD_SEMI_AUTO);
    }else if(ui->rb_auto->isChecked() == true){
        command.append(SCARA_METHOD_GCODE);
    }else if(ui->rb_test->isChecked() == true){
        command.append(SCARA_METHOD_TEST);
    }else if(ui->rb_pick_and_place->isChecked() == true){
        command.append(SCARA_METHOD_PICK_AND_PLACE);
    }else{
        log_console("Please select method");
        log_console("--------------------------");
        return;
    }
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_clear_console_clicked()
{
    ui->tb_console->clear();
}

void MainWindow::on_bt_testmt()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_MOTOR_TEST);
    uint8_t sign = (ui->hs_testmt_sign->value() == 1)?1:0;
    uint8_t pos = (ui->rb_test_mt1->isChecked()==true)?0:
                  (ui->rb_test_mt2->isChecked()==true)?1:
                  (ui->rb_test_mt3->isChecked()==true)?2:
                  (ui->rb_test_mt4->isChecked()==true)?3:0;
    SCARA_TestMode test_mode = SCARA_TestMode(pos*2 + sign);
    QPushButton *obj_sender = (QPushButton*)sender();
    if(obj_sender->objectName() == "bt_start_test"){
       command.append(test_mode);
    }else if(obj_sender->objectName() == "bt_stop_test"){
       command.append(SCARA_TEST_MOTOR_STOP);
    }
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::object_detected(double x, double y, double roll)
{
    QByteArray command;
    command.append(0x28);
    command.append("00");
    command.append(0x01);
    command.append(24);
    int32_t number = (int32_t)(x * 1000000);
    command.append(reinterpret_cast<const char*>(&number), sizeof(number));
    number = (int32_t)(y * 1000000);
    command.append(reinterpret_cast<const char*>(&number), sizeof(number));
    number = (int32_t)(roll* 1000000);
    command.append(reinterpret_cast<const char*>(&number), sizeof(number));
    command.append("00");
    command.append("})");
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_testing_clicked()
{
    GCode_Coordinate_TypeDef t1, t2;
    t1.Feed = 12.45;
    t2 = t1;
    t1.Feed = 45.6;
    qDebug()<<"endd";
}


void MainWindow::on_bt_conveyor_sp_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_SETUP_PNP_CONFIGURE);
    ADD_VALUE(&command, ui->tb_conveyor_sp->text(), SCARA_COR_VALUE_TEXT);
    for(int i = 1; i <= 10; i++){
        if(i == 5){ //move type define
            if(ui->rb_time_constraint->isChecked() == true){
                command.append(DUTY_MODE_INIT_QT);
            }else if(ui->rb_veloc_constraint->isChecked() == true){
                command.append(DUTY_MODE_INIT_QV);
            }
        }
        QLineEdit *tb = this->findChild<QLineEdit*>("tb_p2p_" + QString::number(i));
        ADD_VALUE(&command, tb->text(), SCARA_COR_VALUE_TEXT);
    }
    if(ui->rb_pnp_movJ->isChecked() == true){
        command.append(CMD_MOVE_JOINT);
    }else if(ui->rb_pnp_movL->isChecked() == true){
        command.append(CMD_MOVE_LINE);
    }
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_browse_clicked()
{
    QString file_directory = QFileDialog::getOpenFileName(this, "Open A File", "D:\gcode\\");
    ui->tb_file_dir->setText(file_directory);
}

void MainWindow::on_bt_process_clicked()
{
    gcode->Clear_Data();
    ui->pb_gcode_process->setValue(0);
    Gcode_Decoder_DTC_TypeDef Gcode_DTC;
    try {
       gcode->Init_Current_Data(0, 0, 0, ui->tb_gcode_initial->text().toDouble());
    }  catch (QString exp) {
        log_console("Gcode Initial Speed invalid value");
        return;
    }
    QFile file(ui->tb_file_dir->text());
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        log_console("Cannot open a file");
        return;
    }
    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        Gcode_DTC = gcode->Process_Line(line);
        if(Gcode_DTC != GCODE_OK){
            log_console("Failed to read Gcode file. DTC code: " + QString::number(Gcode_DTC));
            return;
        }
    }
    file.close();

    Gcode_DTC = gcode->Process_Compress_Gcode_Data();
    if(Gcode_DTC == GCODE_OK){
        log_console("Gcode file compresses successfully");
    }else{
        log_console("Failed to compress Gcode file. DTC code: " + QString::number(Gcode_DTC));
        return;
    }

    Gcode_DTC = gcode->Write_Data_To_File("D:/gcode/file.gcode");
    if(Gcode_DTC == GCODE_OK){
        log_console("Gcode file writes successfully");
    }else{
        log_console("Failed to write Gcode file. DTC code: " + QString::number(Gcode_DTC));
    }

    if(ui->rb_gcode_lspb->isChecked() == true){
        Gcode_DTC = gcode->LSPB_Process(ui->tb_limit_angle->text().toDouble());
        gcode->package_data(GCODE_SMOOTH_LSPB);
    }else if(ui->rb_gcode_linear->isChecked() == true){
        Gcode_DTC = gcode->Linear_Process(ui->tb_limit_angle->text().toDouble());
        gcode->package_data(GCODE_LINEAR);
    }

    QByteArray send_packet;
    for(int c = 0; c < gcode->data_packet.count(); c++){
        send_packet.append(gcode->data_packet.at(c));
    }
    qDebug() << "send packet "<< send_packet.toHex();

    mSerial->write(send_packet, send_packet.length());
    log_console("Data has been sent successfully, " + QString::number(send_packet.length()) + " bytes in total");
}

void MainWindow::on_bt_gcode_start_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_GCODE_RUN);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_gcode_stop_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_GCODE_STOP);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_gcode_pause_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_GCODE_PAUSE);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_gcode_resume_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_GCODE_RESUME);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_gcode_configure_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
        qDebug() << command << command.size();
    command.append(COMMAND_TRANSMISION);
        qDebug() << command << command.size();
    command.append(CMD_GCODE_CONFIGURE);
        qDebug() << command << command.size();
    ADD_VALUE(&command, ui->tb_x_offset->text(), SCARA_COR_VALUE_TEXT);
    ADD_VALUE(&command, ui->tb_y_offset->text(), SCARA_COR_VALUE_TEXT);
    ADD_VALUE(&command, ui->tb_z_offset->text(), SCARA_COR_VALUE_TEXT);
    ADD_VALUE(&command, ui->tb_hold_roll_angle->text(), SCARA_COR_VALUE_TEXT);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    qDebug() << command << command.size();
    qDebug() << sizeof(START_CHAR) << sizeof(RECEIVE_END);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_movC1_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_READ_POSITION);
    command.append(READ_REAL_DATA);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
    MovC_ACK = MOVC_TYPE1;
    //process data when current coordinate is received
}
void  MainWindow::MovC_Hanlder(Coordinate_Receive_Handler_TypeDef type, Display_packet data)
{
    double current_x, current_y, target_x, target_y;
    double point_distance, D_distance;
    double input_radius, gradiant, alpha;
    double center_x1, center_y1, center_x2, center_y2;
    double angle_current1, angle_target1, angle_current2, angle_target2;
    double midx, midy;
    double subX, subY;
    current_x = data.RealData.x;
    current_y = data.RealData.y;
    if(type == MOVC_TYPE1){
        target_x = ui->tb_x_cor_c1->text().toDouble();
        target_y = ui->tb_y_cor_c1->text().toDouble();
        double centerX, centerY;
        midx = (current_x + target_x) / 2;
        midy = (current_y + target_y) / 2;
        gradiant = atan2(target_y - current_y, target_x - current_x);
        alpha = M_PI/2 - gradiant;
        input_radius = ui->tb_radius_cor->text().toDouble();
        point_distance = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        //check if input radius is valid or not
        if(2*abs(input_radius) < point_distance -0.001){ //0.001 is safe distance to get the correct condition
            log_console("The input radius is invalid");
            return;
        }
        D_distance = sqrt(input_radius*input_radius - pow(point_distance/2, 2));
        center_x1 = midx - D_distance*cos(alpha);
        center_y1 = midy + D_distance*sin(alpha);
        center_x2 = midx + D_distance*cos(alpha);
        center_y2 = midy - D_distance*sin(alpha);
        angle_current1 = atan2(current_y - center_y1, current_x - center_x1);
        angle_target1 = atan2(target_y - center_y1, target_x - center_x1);
        angle_current2 = atan2(current_y - center_y2, current_x - center_x2);
        angle_target2 = atan2(target_y - center_y2, target_x - center_x2);

        if(input_radius < 0){
            double denta1 = angle_target1 - angle_current1;
            double denta2 = angle_target2 - angle_current2;
            if(ui->rb_aw_c1->isChecked() == true){
                if((denta1 > 0 && abs(denta1) > M_PI) || (denta2 > 0 && abs(denta2) <= M_PI)){
                    CHOOSE_CENTER1;
                }else if((denta1 > 0 && abs(denta1) <= M_PI) || (denta2 > 0 && abs(denta2) > M_PI)){
                    CHOOSE_CENTER2;
                }else if(denta1 < 0 && abs(denta1) <= M_PI){
                    CHOOSE_CENTER1;
                }else if(denta2 < 0 && abs(denta2) <= M_PI){
                    CHOOSE_CENTER2;
                }
            }else if(ui->rb_cw_c1->isChecked() == true){
                if((denta1 < 0 && abs(denta1) > M_PI) || (denta2 < 0 && abs(denta2) <= M_PI)){
                    CHOOSE_CENTER1
                }else if((denta1 < 0 && abs(denta1) <= M_PI) || (denta2 < 0 && abs(denta2) > M_PI)){
                    CHOOSE_CENTER2;
                }else if(denta1 > 0 && abs(denta1) <= M_PI){
                    CHOOSE_CENTER1;
                }else if(denta2 > 0 && abs(denta2) <= M_PI){
                    CHOOSE_CENTER2;
                }
            }
        }else if(input_radius > 0){
            double denta1 = angle_target1 - angle_current1;
            double denta2 = angle_target2 - angle_current2;
            if(ui->rb_aw_c1->isChecked() == true){
                if((denta1 > 0 && abs(denta1) < M_PI) || (denta2 > 0 && abs(denta2) >= M_PI)){
                    CHOOSE_CENTER1;
                }else if((denta1 > 0 && abs(denta1) >= M_PI) || (denta2 > 0 && abs(denta2) < M_PI)){
                    CHOOSE_CENTER2;
                }else if(denta1 < 0 && abs(denta1) >= M_PI){
                    CHOOSE_CENTER1;
                }else if(denta2 < 0 && abs(denta2) >= M_PI){
                    CHOOSE_CENTER2;
                }
            }else if(ui->rb_cw_c1->isChecked() == true){
                if((denta1 < 0 && abs(denta1) < M_PI) || (denta2 < 0 && abs(denta2) >= M_PI)){
                    CHOOSE_CENTER1;
                }else if((denta1 < 0 && abs(denta1) >= M_PI) || (denta2 < 0 && abs(denta2) < M_PI)){
                    CHOOSE_CENTER2;
                }else if(denta1 > 0 && abs(denta1) >= M_PI){
                    CHOOSE_CENTER1;
                }else if(denta2 > 0 && abs(denta2) >= M_PI){
                    CHOOSE_CENTER2;
                }
            }
        }else{
            log_console("Input radius is zero");
            return;
        }
        subX = centerX - current_x;
        subY = centerY - current_y;
    }else if(type == MOVC_TYPE2){
        target_x = ui->tb_x_cor_c2->text().toDouble();
        target_y = ui->tb_y_cor_c2->text().toDouble();
        double radius1, radius2;
        double centerX, centerY;
        centerX = current_x + ui->tb_subpoint_x->text().toDouble();
        centerY = current_y + ui->tb_subpoint_y->text().toDouble();
        radius1 = sqrt(pow(current_x - centerX, 2) + pow(current_y - centerY, 2));
        radius2 = sqrt(pow(target_x - centerX, 2) + pow(target_y - centerY, 2));
        if(abs(radius1 - radius2) > 0.001){
            log_console("Invalid subpoint value, unequal radius");
            return;
        }
        subX = centerX - current_x;
        subY = centerY - current_y;
    }

    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_MOVE_CIRCLE);
    ADD_VALUE(&command, target_x, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, target_y, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, subX, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, subY, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&command, ui->tb_roll_ang_c1->text(), SCARA_COR_VALUE_TEXT);
    if(ui->tb_v_factor->text().toDouble() > 1 || ui->tb_v_factor->text().toDouble() < 0){
        log_console("Invalid for velocity factor");
        return;
    }else{
        ADD_VALUE(&command, ui->tb_v_factor->text(), SCARA_COR_VALUE_TEXT);
    }
    if(ui->rb_cw_c1->isChecked() == true){
        command.append(ARC_CW_TYPE);
    }else if(ui->rb_aw_c1->isChecked() == true){
        command.append(ARC_AW_TYPE);
    }
    if(ui->rb_qva->isChecked() == true) {
        command.append(DUTY_MODE_INIT_QVA);
        ADD_VALUE(&command, ui->tb_a_factor->text(), SCARA_COR_VALUE_TEXT);
    }else if(ui->rb_qvt->isChecked() == true){
        command.append(DUTY_MODE_INIT_QVT);
        ADD_VALUE(&command, ui->tb_time->text(), SCARA_COR_VALUE_TEXT);
    }else if(ui->rb_qt->isChecked() == true){
        command.append(DUTY_MODE_INIT_QT);
        ADD_VALUE(&command, ui->tb_time->text(), SCARA_COR_VALUE_TEXT);
    }
    if(ui->rb_abs->isChecked() == true){
       command.append(DUTY_COORDINATES_ABS);
    }else if(ui->rb_inc->isChecked() == true){
       command.append(DUTY_COORDINATES_REL);
    }
    if(ui->rb_lspb->isChecked() == true){
       command.append(DUTY_TRAJECTORY_LSPB);
    }else if(ui->rb_scur->isChecked() == true){
       command.append(DUTY_TRAJECTORY_SCURVE);
    }else if(ui->rb_linear->isChecked() == true){
        command.append(DUTY_TRAJECTORY_LINEAR);
    }
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());

}

void MainWindow::on_bt_movC2_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_READ_POSITION);
    command.append(READ_REAL_DATA);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
    MovC_ACK = MOVC_TYPE2;
    //process data when current coordinate is received
}

void MainWindow::on_bt_sw_test_clicked()
{
    if(ui->gb_position_read->isEnabled() == true){
        SET_CONTROL_UI_STATE(false);
    }else{
        SET_CONTROL_UI_STATE(true);
    }
}


void MainWindow::closeEvent(QCloseEvent *event)
{
    system_parameter->Save_Configuration();
}


void MainWindow::on_bt_conveyor_refresh_clicked()
{
    ui->cb_conveyor_com->clear();
    const auto list_of_port = QSerialPortInfo::availablePorts();
        for (const QSerialPortInfo &port : list_of_port)
            ui->cb_conveyor_com->addItem(port.portName());
}

void MainWindow::on_bt_conveyor_start_clicked()
{
    QByteArray temper_array;
    temper_array.append('(');
    temper_array.append('\0');
    temper_array.append(')');
    mSerial1->write(temper_array);
}

void MainWindow::on_bt_conveyor_stop_clicked()
{
    QByteArray temper_array;
    temper_array.append('(');
    temper_array.append(0x01);
    temper_array.append(')');
    mSerial1->write(temper_array);
}

void MainWindow::on_bt_conveyor_configure_clicked()
{
    QByteArray temper_array;
    temper_array.append('(');
    temper_array.append(0x02);
    temper_array.append((QChar)ui->tb_conveyor_pulse->text().toInt());
    temper_array.append(')');
    mSerial1->write(temper_array);
}

void MainWindow::on_bt_conveyor_reverse_clicked()
{
    QByteArray temper_array;
    temper_array.append('(');
    temper_array.append(0x03);
    temper_array.append(')');
    mSerial1->write(temper_array);
}

void MainWindow::on_bt_conveyor_con_clicked()
{
    if (ui->bt_conveyor_con->text() == "Connect"){
        ui->bt_conveyor_con->setText("Disconnect");
        ui->bt_conveyor_con->setStyleSheet("background-color:green");
        mSerial1 = new QSerialPort(this);
        mSerial1->setPortName(ui->cb_conveyor_com->currentText());
        mSerial1->setBaudRate(QSerialPort::Baud115200);
        mSerial1->setDataBits(QSerialPort::Data8);
        mSerial1->setParity(QSerialPort::NoParity);
        mSerial1->setStopBits(QSerialPort::OneStop);
        mSerial1->setFlowControl(QSerialPort::NoFlowControl);
        mSerial1->open(QIODevice::WriteOnly);
    }else if(ui->bt_conveyor_con->text() == "Disconnect"){
        ui->bt_conveyor_con->setText("Connect");
        ui->bt_conveyor_con->setStyleSheet("background-color:red");
        mSerial1->close();
        delete mSerial1;
    }
}

void MainWindow::on_tb_conveyor_pulse_textChanged(const QString &arg1)
{
    int ppms = 0;
    try
    {
        ppms = ui->tb_conveyor_pulse->text().toInt();
        double fuzzy_value;
    }
    catch(QString exp)
    {
        return;
    }
    ppms = (ppms > 100) ? 100 : ppms;
    ppms = (ppms < 0) ? 0 : ppms;
    double sps = ppms * 10.0 * 35.5 * 3.141592654 * 21.0 / (20.0 * 200.0) / 4;
    ui->lb_conveyor_real_sp->setText(QString::number(sps) + " mm/s");
}

void MainWindow::on_rb_gcode_lspb_clicked()
{
    SET_GCODE_UI(false);
}

void MainWindow::on_rb_gcode_linear_clicked()
{
    SET_GCODE_UI(true);
}

void MainWindow::on_pushButton_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_STEP_ON_OFF);
    command.append(0x01);
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_pushButton_2_clicked()
{
    QByteArray command;
    command.append(START_CHAR);
    command.append("00");
    command.append(COMMAND_TRANSMISION);
    command.append(CMD_STEP_ON_OFF);
    command.append('\0');
    command.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(command);
    mSerial->write(command, command.length());
}

void MainWindow::on_bt_load_offset_clicked()
{
    ui->tb_x_cor->setText(ui->tb_x_offset->text());
    ui->tb_y_cor->setText(ui->tb_y_offset->text());
    ui->tb_z_cor->setText(ui->tb_z_offset->text());
    ui->tb_roll_ang->setText(ui->tb_hold_roll_angle->text());
    ui->tb_v_factor->setText("0");
    ui->tb_time->setText("2");
    ui->rb_movJ->setChecked(true);
    ui->rb_abs->setChecked(true);
    ui->rb_lspb->setChecked(true);
    ui->rb_qvt->setChecked(true);
}
void MainWindow::DAQ(Display_packet &data)
{
    DAQ_content += d2s(data.RealData.x)+",";
    DAQ_content += d2s(data.RealData.y)+",";
    DAQ_content += d2s(data.RealData.z)+",";
    DAQ_content += d2s(data.RealData.roll)+",";
    DAQ_content += d2s(data.RealData.theta1)+",";
    DAQ_content += d2s(data.RealData.theta2)+",";
    DAQ_content += d2s(data.RealData.D3)+",";
    DAQ_content += d2s(data.RealData.theta4)+",";
    NL(DAQ_content);

}
void MainWindow::on_bt_save_browse_clicked()
{
    QString file_directory = QFileDialog::getExistingDirectory(this, "Open Save Directory");
    ui->tb_save_dir->setText(file_directory);
}

void MainWindow::on_bt_save_data_file_clicked()
{
    QString path = ui->tb_save_dir->text();
    path.append(92);
    path.append(ui->tb_file_name->text());
    QFile file(path);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text)){
        file.write(DAQ_content.toStdString().c_str(), DAQ_content.length());
        file.close();
    }else{
        log_console("Data file failed to save");
        return;
    }
    DAQ_content.clear();
    DAQ_content = "X,Y,Z,Roll,Theta1,Theta2,D3,Theta4"; NL(DAQ_content);
    log_console("Data is logged successfully");
}



