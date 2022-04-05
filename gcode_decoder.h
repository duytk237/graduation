#ifndef GCODE_DECODER_H
#define GCODE_DECODER_H

#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include "define_parameter.h"
#include <QFile>
#include "packet_handler.h"

typedef enum
{
    G00,
    G01,
    G02,
    G03,

    NUM_OF_COMMAND
}GCode_TypeDef;

typedef enum
{
    X_COR,
    Y_COR,
    Z_COR,
    I_COR,
    J_COR,
    FEED,
    Value_Check_Typedef_NUM
}Value_Check_Typedef;
typedef struct
{
    double          X = 0;
    double          Y = 0;
    double          Z = 0;
    double          I = 0;
    double          J = 0;
    double          Feed = 0;
    double          T = 0;
    bool            check[Value_Check_Typedef_NUM] = {false, false, false, false, false, false};
    GCode_TypeDef   Command = G00;
    double          gradiant = 0;
    double          delta_gradiant = 0;
    bool            change_height_border = false;
//    bool            bezier_segment = false;
}GCode_Coordinate_TypeDef;
typedef enum
{
    FIRST_PACKET,                 //these contains in 4 bit low
    LINEAR_TYPE,   //G00, G01     //these contains in 4 bit low
    BEZIER_TYPE,                  //these contains in 4 bit low
    ARC_CW_TYPE,   //G02          //these contains in 4 bit low
    ARC_AW_TYPE,   //G03          //these contains in 4 bit low
    CLUTCH_HEADER_TYPE,           //these contains in 4 bit low

    UP_Z,                         //these contains in 4 bit high
    DOWN_Z,                       //these contains in 4 bit high
    GCODE_LINEAR,                 //these contains in 4 bit high
    GCODE_SMOOTH_LSPB             //these contains in 4 bit high
}Gcode_Packet_Command_TypeDef;
typedef struct{
    double acc[3];
    double constant[2];
    double deacc[3];
    double total_s;
    double veloc = 0;
    double Tf = 0;
    double Ta = 0;
    double Td = 0;
    double Sa = 0;
    double Sd = 0;
}LSPB_Parameter_TypeDef;

class Gcode_Decoder: public QObject
{
    Q_OBJECT
public:
    Gcode_Decoder();
    void Clear_Data();
    Gcode_Decoder_DTC_TypeDef Write_Data_To_File(QString path);
    void LSPB_calculation(double total_s, double veloc, LSPB_Parameter_TypeDef &output);
    void Init_Current_Data(double x, double y, double z, double feed);
    double adjust_gradiant(double raw_value);
    Gcode_Decoder_DTC_TypeDef Process_Line(QString Line);
    Gcode_Decoder_DTC_TypeDef package_data(Gcode_Packet_Command_TypeDef execute_mode);
    Gcode_Decoder_DTC_TypeDef Process_Compress_Gcode_Data();
    Gcode_Decoder_DTC_TypeDef LSPB_Process(double limit_angle);
    Gcode_Decoder_DTC_TypeDef Linear_Process(double limit_angle);
    double calculate_linear_distance(GCode_Coordinate_TypeDef start, GCode_Coordinate_TypeDef end);
    double calculate_circle_distance(GCode_Coordinate_TypeDef start, GCode_Coordinate_TypeDef end);
    double solve_quad(double a, double b, double c);
    QList<GCode_Coordinate_TypeDef> bisectrix_calculation(GCode_Coordinate_TypeDef first, GCode_Coordinate_TypeDef second, GCode_Coordinate_TypeDef third);
    GCode_Coordinate_TypeDef                     current_data;
    QList<GCode_Coordinate_TypeDef>              raw_data;
    QList<GCode_Coordinate_TypeDef>              compact_data;
    QList<QByteArray>                            data_packet;
    std::vector<QList<GCode_Coordinate_TypeDef>> execute_data;
    QList<GCode_Coordinate_TypeDef>              linear_execute_data;
    std::vector<LSPB_Parameter_TypeDef>          clutch_configure_data;
private:
    QString Command_String[4] = {"G00", "G01", "G02", "G03"};
    double lastI, lastJ;
    int sub_point_gcode_smooth = 0;
};

extern Gcode_Decoder *gcode;

#endif // GCODE_DECODER_H
