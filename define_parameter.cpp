#include "define_parameter.h"
#include <QFile>
define_parameter::define_parameter()
{

}

void define_parameter::Convert_And_Append(QByteArray *object_array, QVariant convert_object, TypeDef_Conversion input_type)
{
    switch (input_type) {
        case FLOAT_STRING_VALUE:
        {
            QString temp = convert_object.toString();
            float number = temp.toFloat();
            object_array->append(reinterpret_cast<const char*>(&number), sizeof(number));
        }
        break;
        case FLOAT_VALUE:
        {
            float number = convert_object.toFloat();
            object_array->append(reinterpret_cast<const char*>(&number), sizeof(number));
        }
        break;
        case DOUBLE_STRING_VALUE:
        {
            QString temp = convert_object.toString();
            double number = temp.toDouble();
            object_array->append(reinterpret_cast<const char*>(&number), sizeof(number));
        }
        break;
        case DOUBLE_VALUE:
        {
            double number = convert_object.toDouble();
            object_array->append(reinterpret_cast<const char*>(&number), sizeof(number));
        }
        break;
        case BYTE_VALUE:
        {
            QChar number = convert_object.toChar();
            object_array->append(reinterpret_cast<const char*>(&number), 1);
        }
        break;
        case INT16_VALUE:
        {
            int16_t number = convert_object.toInt();
            object_array->append(reinterpret_cast<const char*>(&number), sizeof(number));
        }
        break;
        case INT32_VALUE:
        {
            int32_t number = convert_object.toInt();
            object_array->append(reinterpret_cast<const char*>(&number), sizeof(number));
        }
        break;
        case SCARA_COR_VALUE_TEXT:
        {
            QString temp = convert_object.toString();
            int32_t number = (int32_t)(temp.toDouble()*DATA_FOWARD_SCALE);
            object_array->append(reinterpret_cast<const char*>(&number), sizeof(number));
        }
        break;
        case SCARA_COR_VALUE_DOUBLE:
        {
            int32_t number = (int32_t)(convert_object.toDouble()*DATA_FOWARD_SCALE);
            object_array->append(reinterpret_cast<const char*>(&number), sizeof(number));
        }
        break;
    }
}
void define_parameter::Save_Configuration()
{
    QString file_content;
    file_content.append(PROGRAM_CONFGIURE_CODE); file_content.append("\n");

    SAVE_CONFIGURE(KEY_SPEED, global_ui->tb_key_setsp->text());
    SAVE_CONFIGURE(MOTOR1_TEST_VALUE, global_ui->tb_m1_pulse->text());
    SAVE_CONFIGURE(MOTOR2_TEST_VALUE, global_ui->tb_m2_pulse->text());
    SAVE_CONFIGURE(MOTOR3_TEST_VALUE, global_ui->tb_m3_pulse->text());
    SAVE_CONFIGURE(MOTOR4_TEST_VALUE, global_ui->tb_m4_pulse->text());
    SAVE_CONFIGURE(OFF_X_VALUE, global_ui->tb_x_offset->text());
    SAVE_CONFIGURE(OFF_Y_VALUE, global_ui->tb_y_offset->text());
    SAVE_CONFIGURE(OFF_Z_VALUE, global_ui->tb_z_offset->text());
    SAVE_CONFIGURE(OFF_ROLL_ANGLE, global_ui->tb_hold_roll_angle->text());
    SAVE_CONFIGURE(GCODE_INITIAL_SP, global_ui->tb_gcode_initial->text());
    if(global_ui->rb_gcode_lspb->isChecked() == true){
        SAVE_CONFIGURE(GCODE_SMOOTH_OPTION, "1");
    }else if(global_ui->rb_gcode_linear->isChecked() == true){
        SAVE_CONFIGURE(GCODE_SMOOTH_OPTION, "0");
    }
    SAVE_CONFIGURE(GCODE_SMOOTH_VALUE, global_ui->tb_limit_angle->text());

    SAVE_CONFIGURE(CONVEYOR_SP           , global_ui->tb_conveyor_sp->text());
    SAVE_CONFIGURE(DOWN_TIME_ON_SLOT     , global_ui->tb_p2p_1->text());
    SAVE_CONFIGURE(DOWN_TIME_ON_OBJECT   , global_ui->tb_p2p_2->text());
    SAVE_CONFIGURE(UP_TIME_ON_OBJECT     , global_ui->tb_p2p_3->text());
    SAVE_CONFIGURE(UP_TIME_ON_SLOT       , global_ui->tb_p2p_4->text());
    SAVE_CONFIGURE(MOVE_TIME             , global_ui->tb_p2p_5->text());
    SAVE_CONFIGURE(ATTACH_TIME           , global_ui->tb_p2p_6->text());
    SAVE_CONFIGURE(DETACH_TIME           , global_ui->tb_p2p_7->text());
    SAVE_CONFIGURE(UP_HEIGHT             , global_ui->tb_p2p_8->text());
    SAVE_CONFIGURE(DOWN_HEIGHT_ON_OBJECT , global_ui->tb_p2p_9->text());
    SAVE_CONFIGURE(DOWN_HEIGHT_ON_SLOT   , global_ui->tb_p2p_10->text());
    if(global_ui->rb_pnp_movJ->isChecked() == true){
        SAVE_CONFIGURE(PNP_MOVE_OPTION   , "1");
    }else if(global_ui->rb_pnp_movL->isChecked() == false){
        SAVE_CONFIGURE(PNP_MOVE_OPTION   , "0");
    }
    if(global_ui->rb_time_constraint->isChecked() == true){
        SAVE_CONFIGURE(PNP_MOVE_TYPE, "1");
    }else if(global_ui->rb_veloc_constraint->isChecked() == true){
        SAVE_CONFIGURE(PNP_MOVE_TYPE, "0");
    }
    SAVE_CONFIGURE(LOG_FILE_DIR          , global_ui->tb_save_dir->text());
    SAVE_CONFIGURE(LOG_FILE_NAME         , global_ui->tb_file_name->text());
    QString path = QCoreApplication::applicationDirPath();
    QFile file("configuration.txt");
    if(file.open(QIODevice::WriteOnly | QIODevice::Text)){
        file.write(file_content.toStdString().c_str(), file_content.length());
        file.close();
    }else{
        return;
    }
}
void define_parameter::Load_Configuration()
{
    QFile file("configuration.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        log_console("Cannot open a file");
        return;
    }
    int count_start = 1;
    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        if(count_start == 1){
            if(line.startsWith(PROGRAM_CONFGIURE_CODE)){
                count_start++;
            }else{
                break;
            }
        }else{
            QList<QString> component_list = line.split(':');
            Save_Configuration_TypeDef ident_index = (Save_Configuration_TypeDef)line.split(':').at(0).toInt();
            QString value;
            for(int c = 1; c < component_list.count(); c++){
                value += component_list.at(c);
            }
            switch (ident_index) {
            case KEY_SPEED             :
                global_ui->tb_key_setsp->setText(value);
            break;
            case MOTOR1_TEST_VALUE     :
                global_ui->tb_m1_pulse->setText(value);
            break;
            case MOTOR2_TEST_VALUE     :
                global_ui->tb_m2_pulse->setText(value);
            break;
            case MOTOR3_TEST_VALUE     :
                global_ui->tb_m3_pulse->setText(value);
            break;
            case MOTOR4_TEST_VALUE     :
                global_ui->tb_m4_pulse->setText(value);
            break;
            case OFF_X_VALUE           :
                global_ui->tb_x_offset->setText(value);
            break;
            case OFF_Y_VALUE           :
                global_ui->tb_y_offset->setText(value);
            break;
            case OFF_Z_VALUE           :
                global_ui->tb_z_offset->setText(value);
            break;
            case OFF_ROLL_ANGLE        :
                global_ui->tb_hold_roll_angle->setText(value);
            break;
            case GCODE_INITIAL_SP      :
                global_ui->tb_gcode_initial->setText(value);
            break;
            case GCODE_SMOOTH_OPTION   :
                if(value == "1"){
                    global_ui->rb_gcode_lspb->setChecked(true);
                }else if(value == "0"){
                    global_ui->rb_gcode_linear->setChecked(false);
                }
            break;
            case GCODE_SMOOTH_VALUE    :
                global_ui->tb_limit_angle->setText(value);
            break;
            case CONVEYOR_SP           :
                global_ui->tb_conveyor_sp->setText(value);
            break;
            case DOWN_TIME_ON_SLOT     :
                global_ui->tb_p2p_1->setText(value);
            break;
            case DOWN_TIME_ON_OBJECT   :
                global_ui->tb_p2p_2->setText(value);
            break;
            case UP_TIME_ON_OBJECT     :
                global_ui->tb_p2p_3->setText(value);
            break;
            case UP_TIME_ON_SLOT       :
                global_ui->tb_p2p_4->setText(value);
            break;
            case MOVE_TIME             :
                global_ui->tb_p2p_5->setText(value);
            break;
            case ATTACH_TIME           :
                global_ui->tb_p2p_6->setText(value);
            break;
            case DETACH_TIME           :
                global_ui->tb_p2p_7->setText(value);
            break;
            case UP_HEIGHT             :
                global_ui->tb_p2p_8->setText(value);
            break;
            case DOWN_HEIGHT_ON_OBJECT :
                global_ui->tb_p2p_9->setText(value);
            break;
            case DOWN_HEIGHT_ON_SLOT   :
                global_ui->tb_p2p_10->setText(value);
            break;
            case PNP_MOVE_OPTION       :
                if(value == "1"){
                    global_ui->rb_pnp_movJ->setChecked(true);
                }else if(value == "0"){
                    global_ui->rb_pnp_movL->setChecked(true);
                }
            break;
            case PNP_MOVE_TYPE         :
                if(value == "1"){
                    global_ui->rb_time_constraint->setChecked(true);
                }else if(value == "0"){
                    global_ui->rb_veloc_constraint->setChecked(true);
                }
            break;
            case LOG_FILE_DIR:
                global_ui->tb_save_dir->setText(value);
            break;
            case LOG_FILE_NAME:
                global_ui->tb_file_name->setText(value);
            break;
            default:
            break;
            }
        }
    }
    file.close();
}
Ui::MainWindow *global_ui;
define_parameter *system_parameter = new define_parameter();
