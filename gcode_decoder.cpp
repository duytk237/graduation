#include "gcode_decoder.h"

Gcode_Decoder::Gcode_Decoder()
{

}

void Gcode_Decoder::Clear_Data()
{
    raw_data.clear();
    compact_data.clear();
    data_packet.clear();
    execute_data.clear();
    clutch_configure_data.clear();
    linear_execute_data.clear();
}
Gcode_Decoder_DTC_TypeDef Gcode_Decoder::Process_Line(QString Line) // save at raw_data
{
    if(Line.startsWith("G")){
        GCode_Coordinate_TypeDef temper_coordinate;
        QString temper_line = Line.split("(").at(0);
        if(temper_line.contains(')')){
            //Gcode error line format
            return GCODE_ERROR_LINE_FORMAT;
        }
        QStringList temper_string = temper_line.split(" "); // seperate the comment section that start with '('

        QString component = "";

        //categorize command
        for(int i = 0; i <= 3; i++){
            if(temper_string.at(0) == Command_String[i]){
                temper_coordinate.Command = (GCode_TypeDef)i;
                break;
            }
        }

        //find X coordinate
        int index = 0;
        index = temper_string.indexOf(QRegExp("^X.+"));
        if(index != -1) {
            component = temper_string.at(index);
            if(!component.isEmpty()){
                temper_coordinate.X = component.mid(1, component.length() - 1).toDouble();
            }
        }else{
            temper_coordinate.X = raw_data.last().X;
        }

        //find Y coordinate
        index = temper_string.indexOf(QRegExp("^Y.+"));
        if(index != -1) {
            component = temper_string.at(index);
            if(!component.isEmpty()){
                temper_coordinate.Y = component.mid(1, component.length() - 1).toDouble();
            }
        }else{
            temper_coordinate.Y = raw_data.last().Y;
        }

        //find Z coordinate
        index = temper_string.indexOf(QRegExp("^Z.+"));
        if(index != -1) {
            component = temper_string.at(index);
            if(!component.isEmpty()){
                temper_coordinate.Z = component.mid(1, component.length() - 1).toDouble();
            }
        }else{
            temper_coordinate.Z = raw_data.last().Z;
        }

        //find Feed rate
        index = temper_string.indexOf(QRegExp("^F.+"));
        if(index != -1) {
            component = temper_string.at(index);
            if(!component.isEmpty()){
                temper_coordinate.Feed = component.mid(1, component.length() - 1).toDouble();
            }
        }else{
            temper_coordinate.Feed = raw_data.last().Feed;
        }

        temper_coordinate.check[X_COR] = true;
        temper_coordinate.check[Y_COR] = true;
        temper_coordinate.check[Z_COR] = true;
        temper_coordinate.check[FEED] = true;

        //find I and J coordinate if the command is G02 or G03
        if(temper_coordinate.Command == G02 || temper_coordinate.Command == G03){
            index = temper_string.indexOf(QRegExp("^I.+"));
            if(index != -1) {
                component = temper_string.at(index);
                if(!component.isEmpty()){
                    temper_coordinate.check[I_COR] = true;
                    temper_coordinate.I = component.mid(1, component.length() - 1).toDouble();
                    lastI = temper_coordinate.I;
                }
            }else{
                temper_coordinate.I = lastI;
            }

            index = temper_string.indexOf(QRegExp("^J.+"));
            if(index != -1) {
                component = temper_string.at(index);
                if(!component.isEmpty()){
                    temper_coordinate.check[J_COR] = true;
                    temper_coordinate.J = component.mid(1, component.length() - 1).toDouble();
                    lastJ = temper_coordinate.J;
                }
            }else{
                temper_coordinate.J = lastJ;
            }
            temper_coordinate.check[I_COR] = true;
            temper_coordinate.check[J_COR] = true;
        }
        raw_data.append(temper_coordinate);
    }
    return GCODE_OK;
}

Gcode_Decoder_DTC_TypeDef Gcode_Decoder::Process_Compress_Gcode_Data()
{
    //check if current data has been init or not
    if(current_data.check[X_COR] == false || current_data.check[Y_COR] == false || current_data.check[Z_COR] == false || current_data.check[FEED] == false)
        return CURRENT_DATA_UNINIT;
    int count = 1;
    int check_count = -2;
    Gcode_Compress_State_TypeDef current_state = ROUTE_COMMAND;
    //GCode_Coordinate_TypeDef  last_data = current_data;
    bool switch_flag = false;
    double current_s = 0;
    while(count < raw_data.count()){
        switch (current_state) {
        case ROUTE_COMMAND:{
            if(raw_data.at(count).Command == G02 || raw_data.at(count).Command == G03){
                current_state = STATE_CIRCLE;
            }else if(raw_data.at(count).Command == G00 || raw_data.at(count).Command == G01){
                current_state = STATE_LINEAR;
            }
        }
        break;
        case STATE_LINEAR:{
            current_s = calculate_linear_distance(current_data, raw_data.at(count));
            if(current_s / current_data.Feed > 0*MINIMUM_LINEAR_TIME_FRAME){
                raw_data[count].Command = G01;
                compact_data.append(raw_data.at(count));
                current_data = raw_data.at(count);
                current_state = ROUTE_COMMAND;
                switch_flag = false;
            }else{
                current_state = ROUTE_COMMAND;
            }
            count++;
        }
        break;
        case STATE_CIRCLE:{
            current_s = calculate_circle_distance(raw_data.at(count-1), raw_data.at(count));
            if(current_s / current_data.Feed > MINIMUM_CIRCLE_TIME_FRAME){
                compact_data.removeLast();
                compact_data.append(raw_data.at(count-1));
                compact_data.append(raw_data.at(count));
                if(switch_flag == true){
                    switch_flag = false;
                    compact_data[compact_data.count()-2].Command = G01; //convert the the last previous command into G01 in case it G02 or G03
                }
                current_data = raw_data.at(count);
                count++;
                current_state = ROUTE_COMMAND;
            }else{
                switch_flag = true;
                current_state = STATE_LINEAR;
            }
        }
        break;
        }
        check_count++; //count value must end before check_count to ensure successful compression
        if(check_count > raw_data.count()*100) return GCODE_PROCESS_ERROR; //make sure the algorithm wont stuck in the loop
    }
    //compact_data = raw_data;
    return GCODE_OK;
}

Gcode_Decoder_DTC_TypeDef Gcode_Decoder::package_data(Gcode_Packet_Command_TypeDef execute_mode)
{

    QByteArray temper_array;
    double Min_Z, Max_Z;
    std::vector<double> Z_cor;
    for(int i = 0; i < compact_data.count(); i++){
        Z_cor.push_back(compact_data.at(i).Z);
    }
    std::sort(Z_cor.begin(), Z_cor.end(), [](double x, double y){ return (x<y);});
    Min_Z = Z_cor.at(0);
    Max_Z = *(Z_cor.end()-1);
    qDebug() << Min_Z << Max_Z;
    qDebug() << "int32_t " <<sizeof(int32_t)<< "int 16_t" << sizeof(int16_t)<< "FILE_TRANSMISION "<<sizeof(FILE_TRANSMISION);
    //init first packet
    temper_array.append(START_CHAR);
    temper_array.append("00");
    temper_array.append(FILE_TRANSMISION);
    temper_array.append(execute_mode << 4 | FIRST_PACKET);
//    if(execute_mode == GCODE_LINEAR){
//        ADD_VALUE(&temper_array, global_ui->tb_gcode_wc->text(), SCARA_COR_VALUE_TEXT);
//    }
    ADD_VALUE(&temper_array, Min_Z, SCARA_COR_VALUE_DOUBLE);
    ADD_VALUE(&temper_array, Max_Z, SCARA_COR_VALUE_DOUBLE);

    if(execute_mode == GCODE_LINEAR){
        ADD_VALUE(&temper_array, linear_execute_data.count(), INT32_VALUE);
    }else if(execute_mode == GCODE_SMOOTH_LSPB){
        ADD_VALUE(&temper_array, compact_data.count() - 1 - sub_point_gcode_smooth, INT32_VALUE);
    }   
    qDebug() << cdompact_data.count() - 1 - sub_point_gcode_smooth;
    temper_array.append(RECEIVE_END);
    PACKET_DEFINE_LENGTH(temper_array);
    data_packet.append(temper_array);                       // packet1 -> clear
    temper_array.clear();
    if(execute_mode == GCODE_LINEAR){
        //init 10 point in a row into 1 sending packet
        int count = 0;
        Gcode_Packet_Command_TypeDef tool_height;
        for(int i = 0; i < linear_execute_data.count(); i++){
            if(count == 0){
                temper_array.append(START_CHAR);
                temper_array.append("00");
                temper_array.append(FILE_TRANSMISION);
            }

            if(abs(linear_execute_data.at(i).Z - Max_Z) < 0.001){
                tool_height = UP_Z;
            }else if(abs(linear_execute_data.at(i).Z - Min_Z) < 0.001){
                tool_height = DOWN_Z;
            }else{
                return UNMATCH_Z_HEIGHT;
            }

            switch(linear_execute_data.at(i).Command){
                case G00:
                case G01:{
//                    if(linear_execute_data.at(i).bezier_segment == true){
//                        temper_array.append(tool_height << 4 | BEZIER_TYPE);
//                        ADD_VALUE(&temper_array, linear_execute_data.at(i).I, SCARA_COR_VALUE_DOUBLE);
//                    }else{
//                        temper_array.append(tool_height << 4 | LINEAR_TYPE);
//                    }
                    temper_array.append(tool_height << 4 | LINEAR_TYPE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).X, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).Y, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).Feed, SCARA_COR_VALUE_DOUBLE);
                }
                break;
                case G02:{
                    temper_array.append(tool_height << 4 | ARC_CW_TYPE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).X, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).Y, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).Feed, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).I, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).J, SCARA_COR_VALUE_DOUBLE);
                }
                break;
                case G03:{
                    temper_array.append(tool_height << 4 | ARC_AW_TYPE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).X, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).Y, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).Feed, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).I, SCARA_COR_VALUE_DOUBLE);
                    ADD_VALUE(&temper_array, linear_execute_data.at(i).J, SCARA_COR_VALUE_DOUBLE);
                }
                break;
            }

            if(count == 9){
                temper_array.append(RECEIVE_END);
                PACKET_DEFINE_LENGTH(temper_array);
                data_packet.append(temper_array);
                temper_array.clear();
            }
            count = (count+1)%10;
        }
        if(count != 0){ //uneven element in 1 packet
            temper_array.append(RECEIVE_END);
            PACKET_DEFINE_LENGTH(temper_array);
            data_packet.append(temper_array);
            temper_array.clear();
        }
    }else if(execute_mode == GCODE_SMOOTH_LSPB){
        //init 10 point in a row into 1 sending packet
        int count = 0;
        Gcode_Packet_Command_TypeDef tool_height;
        for(int i = 0; i < execute_data.size(); i++){
            if(count == 0){
                temper_array.append(START_CHAR);
                temper_array.append("00");
                temper_array.append(FILE_TRANSMISION);
            }
            temper_array.append(CLUTCH_HEADER_TYPE);
            ADD_VALUE(&temper_array, clutch_configure_data.at(i).total_s, SCARA_COR_VALUE_DOUBLE);
            ADD_VALUE(&temper_array, clutch_configure_data.at(i).veloc, SCARA_COR_VALUE_DOUBLE);
            ADD_VALUE(&temper_array, execute_data.at(i).at(0).Feed, SCARA_COR_VALUE_DOUBLE);
            int start_index = 1;
            if(execute_data.at(i).at(0).change_height_border == true && i != 0){
                start_index = 0;
            }
            for(int j = start_index; j < execute_data.at(i).size(); j++){
                if(count == 0 && j != start_index){
                    temper_array.append(START_CHAR);
                    temper_array.append("00");
                    temper_array.append(FILE_TRANSMISION);
                }
                if(abs(execute_data.at(i).at(j).Z - Max_Z) < 0.001){
                    tool_height = UP_Z;
                }else if(abs(execute_data.at(i).at(j).Z - Min_Z) < 0.001){
                    tool_height = DOWN_Z;
                }else{
                    return UNMATCH_Z_HEIGHT;
                }
                switch(execute_data.at(i).at(j).Command){
                    case G00:
                    case G01:{
                        temper_array.append(tool_height << 4 | LINEAR_TYPE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).X, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).Y, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).T, SCARA_COR_VALUE_DOUBLE);
                    }
                    break;
                    case G02:{
                        temper_array.append(tool_height << 4 | ARC_CW_TYPE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).X, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).Y, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).T, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).I, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).J, SCARA_COR_VALUE_DOUBLE);
                    }
                    break;
                    case G03:{
                        temper_array.append(tool_height << 4 | ARC_AW_TYPE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).X, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).Y, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).T, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).I, SCARA_COR_VALUE_DOUBLE);
                        ADD_VALUE(&temper_array, execute_data.at(i).at(j).J, SCARA_COR_VALUE_DOUBLE);
                    }
                    break;
                }

                if(count == 19){
                    temper_array.append(RECEIVE_END);
                    PACKET_DEFINE_LENGTH(temper_array);
                    data_packet.append(temper_array);
                    temper_array.clear();                   // packet 10 position -> clear
                }
                count = (count+1)%20;
            }
        }
        if(count != 0){ //uneven element in 1 packet
            temper_array.append(RECEIVE_END);
            PACKET_DEFINE_LENGTH(temper_array);
            data_packet.append(temper_array);
            temper_array.clear();
        }
    }


    return GCODE_OK;

}
double Gcode_Decoder::calculate_circle_distance(GCode_Coordinate_TypeDef start, GCode_Coordinate_TypeDef end)
{
    double x_center, y_center, start_x1, start_y1, end_x1, end_y1;
    double angle_start, angle_end, radius, denta_angle;
    radius   = sqrt(end.I*end.I + end.J*end.J);
    x_center = start.X + end.I;
    y_center = start.Y + end.J;
    start_x1 = -end.I;
    start_y1 = -end.J;
    end_x1   = end.X - x_center;
    end_y1   = end.Y - y_center;
    angle_start = atan2(start_y1, start_x1);
    angle_end   = atan2(end_y1, end_x1);
    denta_angle = angle_end - angle_start;
    switch (end.Command) {
        case G02:
            if(denta_angle > 0){
                return (2*M_PI - abs(denta_angle))*radius;
            }else{
                return abs(denta_angle)*radius;
            }
        break;
        case G03:
            if(denta_angle > 0){
                return  abs(denta_angle)*radius;
            }else{
                return (2*M_PI - abs(denta_angle))*radius;
            }
        break;
    }
}

double Gcode_Decoder::calculate_linear_distance(GCode_Coordinate_TypeDef start, GCode_Coordinate_TypeDef end)
{
    //start coordinate must be init
    double denta_x, denta_y, denta_z;
    denta_x = end.X - start.X;
    denta_y = end.Y - start.Y;
    denta_z = end.Z - start.Z;
    return sqrt(denta_x*denta_x + denta_y*denta_y + denta_z*denta_z);
}
Gcode_Decoder_DTC_TypeDef Gcode_Decoder::Linear_Process(double limit_angle)
{
    limit_angle = fabs(limit_angle*M_PI/180);
    //calculate each clutch gradiant and its delta, seperate into each clutch with insignificant delta_graditant
    QList<GCode_Coordinate_TypeDef> current_clutch;
    bool start = true;
    int count = -1;
    for(int i = 0; i < compact_data.size(); i++){
        if(start == true){
            start = false;
            count++;
            execute_data.push_back(current_clutch);
            execute_data[0].push_back(compact_data.at(0));
            compact_data[0].gradiant = atan2(compact_data.at(1).Y - compact_data.at(0).Y, compact_data.at(1).X - compact_data.at(0).X);
        }else{
            if(compact_data.at(i).Z != compact_data.at(i-1).Z){
                compact_data[i].delta_gradiant = 0;
                compact_data[i].gradiant = atan2(compact_data.at(i+1).Y - compact_data.at(i).Y, compact_data.at(i+1).X - compact_data.at(i).X);
            }else if(fabs(compact_data.at(i).Z - compact_data.at(i-1).Z) < 0.0001){
                compact_data[i].gradiant = atan2(compact_data.at(i).Y - compact_data.at(i-1).Y, compact_data.at(i).X - compact_data.at(i-1).X);
                compact_data[i].delta_gradiant = adjust_gradiant(compact_data.at(i).gradiant - compact_data.at(i-1).gradiant);
                if(compact_data.at(i).delta_gradiant > limit_angle &&
                        compact_data.at(i).Command != G02 && compact_data.at(i).Command != G03 &&
                        compact_data.at(i-1).Command != G02 && compact_data.at(i-1).Command != G03){
                    execute_data.push_back(current_clutch);
                    count++;
                }
            }
            execute_data[count].push_back(compact_data.at(i));
        }
    }

    //curve all the significant delta gradiant segment
    for(int i = 0; i < execute_data.size(); i++){
        if(i != 0){
            QList<GCode_Coordinate_TypeDef> temper_list;
            if(execute_data.at(i-1).size() == 1){
                temper_list = bisectrix_calculation(execute_data.at(i-2).at(execute_data.at(i-2).size()-1), execute_data.at(i-1).at(0), execute_data.at(i).at(0));
            }else{
                temper_list = bisectrix_calculation(execute_data.at(i-1).at(execute_data.at(i-1).size()-2), execute_data.at(i-1).at(execute_data.at(i-1).size()-1), execute_data.at(i).at(0));
            }
            linear_execute_data.pop_back();
            linear_execute_data.push_back(temper_list.at(0));
            linear_execute_data.push_back(temper_list.at(1));
        }
        for(int j = 0; j < execute_data.at(i).size(); j++){
            linear_execute_data.push_back(execute_data.at(i).at(j));
        }
    }
    return GCODE_OK;
}

Gcode_Decoder_DTC_TypeDef Gcode_Decoder::LSPB_Process(double limit_angle)
{
    limit_angle = fabs(limit_angle*M_PI/180);
    //seperate packet with different Z height
    std::vector<QList<GCode_Coordinate_TypeDef>> process_data;
    QList<GCode_Coordinate_TypeDef> current_clutch;
    double last_z;
    bool start = true;
    for(int t = 0; t < compact_data.size(); t++){
        if(start == true){      // su dung cho t = 0
            start = false;
            process_data.push_back(current_clutch);                                 //add new vector
            process_data[process_data.size() - 1].push_back(compact_data.at(t));    //append data into new vector created
            last_z = compact_data.at(t).Z;
        }else{                  // su dung cho cac t con lai
            if(compact_data.at(t).Z != last_z){
                process_data.push_back(current_clutch);                             // if Z changes -> add new vector
                process_data[process_data.size() - 1].push_back(compact_data.at(t));//append data into new vector created
            }else{
                process_data[process_data.size() - 1].push_back(compact_data.at(t));// if Z don't changes -> append data into vector created before
            }
            last_z = compact_data.at(t).Z;
        }
    }
    // process_data contains List with coordinates with same Z and Vetor with another Z
    //eliminate all the group have the size of 1
    int temper_num = 0;
    sub_point_gcode_smooth = 0; //this will be used after to get the correct total number of point in Gcode smooth mode
    do{ //remove each vector has size == 1
        if(process_data.at(temper_num).size() == 1){
            process_data.erase(process_data.begin() + temper_num);
            sub_point_gcode_smooth++;
        }else{
            temper_num++;
        }
    }while(temper_num < process_data.size());
    int count = -1;
    //calculate each clutch gradiant and its delta, seperate into each clutch with insignificant delta_graditant
    for(int c1 = 0; c1 < process_data.size(); c1++){ //interrate each height clutch
        execute_data.push_back(current_clutch);
        count++;
        int change_height_index = execute_data.size() - 1;
        for(int c2 = 0; c2 < process_data.at(c1).size(); c2++){
            if(c2 == 0){
                process_data[c1][c2].gradiant = 0;
                process_data[c1][c2].delta_gradiant = 0;
                execute_data[count].push_back(process_data.at(c1).at(c2));
            }else if(c2 == 1){
                process_data[c1][1].gradiant = 0;//atan2(process_data[c1][1].Y - process_data[c1][0].Y,process_data[c1][1].X - process_data[c1][0].X);
                process_data[c1][1].delta_gradiant = 0;//fabs(process_data[c1][1].gradiant);
                execute_data[count].push_back(process_data[c1][1]);
            }else{
                process_data[c1][c2].gradiant = 0; //atan2(process_data[c1][c2].Y - process_data[c1][c2-1].Y,process_data[c1][c2].X - process_data[c1][c2-1].X);
                process_data[c1][c2].delta_gradiant = 0; //adjust_gradiant(process_data[c1][c2].gradiant - process_data[c1][c2-1].gradiant);//calculate abs if > 2pi -> 2pi - ans
                if(false){ //process_data[c1][c2].delta_gradiant > limit_angle){ // add new vector if delta_gradient is lagre
                    execute_data.push_back(current_clutch);
                    count++;
                    execute_data[count].push_back(process_data.at(c1).at(c2-1));
                    execute_data[count].push_back(process_data.at(c1).at(c2));
                }else{
                    execute_data[count].push_back(process_data.at(c1).at(c2));
                }
            }
        }
        execute_data[change_height_index].first().change_height_border = true;
    }

    //calculate LSPB for each clutch in execute_data
    LSPB_Parameter_TypeDef current_lspb;
    for(int i = 0; i < execute_data.size(); i++){
        //calculate the total_s in 1 clutch
        double total_s = 0;
        for(int j = 1; j < execute_data.at(i).size(); j++){
            if(execute_data.at(i).at(j).Command == G02 || execute_data.at(i).at(j).Command == G03){
                total_s += calculate_circle_distance(execute_data.at(i).at(j-1), execute_data.at(i).at(j));
            }else if(execute_data.at(i).at(j).Command == G01 || execute_data.at(i).at(j).Command == G00){
                total_s += calculate_linear_distance(execute_data.at(i).at(j-1), execute_data.at(i).at(j));
            }
        }
        LSPB_calculation(total_s, execute_data.at(i).at(1).Feed, current_lspb);
        clutch_configure_data.push_back(current_lspb);
    }

    //calculate move time between point in each clutchs
    for(int i = 0; i < execute_data.size(); i++){
        double count_s = 0;
        for(int j = 1; j < execute_data.at(i).size(); j++){
            if(execute_data.at(i).at(j).Command == G02 || execute_data.at(i).at(j).Command == G03){
                count_s += calculate_circle_distance(execute_data.at(i).at(j-1), execute_data.at(i).at(j));
            }else if(execute_data.at(i).at(j).Command == G01 || execute_data.at(i).at(j).Command == G00){
                count_s += calculate_linear_distance(execute_data.at(i).at(j-1), execute_data.at(i).at(j));
            }
            if(count_s >= 0 && count_s < clutch_configure_data.at(i).Sa){
                execute_data[i][j].T = sqrt(count_s/clutch_configure_data.at(i).acc[0]);
            }else if(count_s >= clutch_configure_data.at(i).Sa && count_s < clutch_configure_data.at(i).Sd){
                execute_data[i][j].T = (count_s - clutch_configure_data.at(i).constant[1])/clutch_configure_data.at(i).constant[0];
            }else if(count_s >= clutch_configure_data.at(i).Sd && count_s <= clutch_configure_data.at(i).total_s){
                execute_data[i][j].T = solve_quad(clutch_configure_data.at(i).deacc[0],
                                                  clutch_configure_data.at(i).deacc[1],
                                                  clutch_configure_data.at(i).deacc[2] - count_s);
            }
        }
    }

    return GCODE_OK;
}

void Gcode_Decoder::LSPB_calculation(double total_s, double veloc, LSPB_Parameter_TypeDef &output)
{
    double acc;
    output.total_s = total_s;
    output.Tf = 1.2*total_s/veloc;
    output.Ta = output.Tf - total_s/veloc;
    output.Td = output.Tf - output.Ta;
    acc = veloc/output.Ta;
    output.veloc = veloc;
    //coefficents of accelaration part
    output.acc[0] = 0.5*acc;
    output.acc[1] = 0;
    output.acc[2] = 0;

    //coefficents of constant part
    output.constant[0] = veloc;
    output.constant[1] = -0.5*output.Ta*veloc;

    //coefficents of deaccelaration part
    output.deacc[0] = -0.5*acc;
    output.deacc[1] = veloc + acc*output.Td;
    output.deacc[2] = total_s - output.Tf*(2*veloc + 2*acc*output.Td - acc*output.Tf)/2;

    output.Sa = output.acc[0]*pow(output.Ta, 2);
    output.Sd = output.constant[0]*output.Td + output.constant[1];
}
QList<GCode_Coordinate_TypeDef> Gcode_Decoder::bisectrix_calculation(GCode_Coordinate_TypeDef first, GCode_Coordinate_TypeDef second, GCode_Coordinate_TypeDef third)
{
    double line_in = calculate_linear_distance(first, second);
    double line_out = calculate_linear_distance(second, third);
    double min_line;
    min_line = (line_in < line_out)?line_in:line_out;
    double insight_angle = M_PI - third.delta_gradiant;
    double in_grad = second.gradiant;
    double out_grad = third.gradiant;
    double reverse_in_grad = in_grad + M_PI;
    reverse_in_grad = (reverse_in_grad >= M_PI)?-(2*M_PI - reverse_in_grad):reverse_in_grad;
    double bisec_grad;
    if(fabs(reverse_in_grad - out_grad) >= M_PI){
        if(reverse_in_grad == M_PI){
            reverse_in_grad = -M_PI;
        }else if(out_grad == M_PI){
            out_grad = -M_PI;
        }
    }
    bisec_grad = (reverse_in_grad + out_grad)/2;
    double cutoff_dis = min_line - min_line*global_ui->tb_gcode_wc->text().toDouble();
    double hypotenuse = cutoff_dis/cos(insight_angle/2);
    double intersect_x = second.X;
    double intersect_y = second.Y;
    double CenterX = intersect_x + hypotenuse*cos(bisec_grad);
    double CenterY = intersect_y + hypotenuse*sin(bisec_grad);
    double EndX = intersect_x + cutoff_dis*cos(out_grad);
    double EndY = intersect_y + cutoff_dis*sin(out_grad);
    double StartX = intersect_x + cutoff_dis*cos(reverse_in_grad);
    double StartY = intersect_y + cutoff_dis*sin(reverse_in_grad);
    QList<GCode_Coordinate_TypeDef> temper_list({first, second});
    temper_list[0].X = StartX;
    temper_list[0].Y = StartY;
    temper_list[0].Command = G00;
    temper_list[1].X = EndX;
    temper_list[1].Y = EndY;
    temper_list[1].I = CenterX - StartX;
    temper_list[1].J = CenterY - StartY;
    double angleS = atan2(StartY-CenterY, StartX-CenterX);
    double angleE = atan2(EndY-CenterY, EndX-CenterX);
    double delta_angle = angleE - angleS;
    if(fabs(delta_angle) > M_PI){
        if(delta_angle > 0){
            temper_list[1].Command = G02;
        }else{
            temper_list[1].Command = G03;
        }
    }else{
        if(delta_angle > 0){
            temper_list[1].Command = G03;
        }else{
            temper_list[1].Command = G02;
        }
    }
    return temper_list;
}
void Gcode_Decoder::Init_Current_Data(double x, double y, double z, double feed)
{
    current_data.X = x;
    current_data.Y = y;
    current_data.Z = z;
    current_data.Feed = feed;
    current_data.check[X_COR] = true;
    current_data.check[Y_COR] = true;
    current_data.check[Z_COR] = true;
    current_data.check[FEED] = true;
    current_data.Command = G00;
    raw_data.append(current_data);
}

Gcode_Decoder_DTC_TypeDef Gcode_Decoder::Write_Data_To_File(QString path)
{
    QString file_content = "";
    for(int i = 0; i < compact_data.count(); i++){
        switch (compact_data.at(i).Command) {
            case G00:
                file_content.append("G0" + QString::number(compact_data.at(i).Command) + " X" +
                               QString::number(compact_data.at(i).X) + " Y" + QString::number(compact_data.at(i).Y)
                               + " Z" + QString::number(compact_data.at(i).Z));
                file_content.append("\r");
            break;
            case G01:
                file_content.append("G0" + QString::number(compact_data.at(i).Command) + " X" +
                           QString::number(compact_data.at(i).X) + " Y" + QString::number(compact_data.at(i).Y)
                           + " Z" + QString::number(compact_data.at(i).Z) + " F" + QString::number(compact_data.at(i).Feed));
                file_content.append("\r");
            break;
            case G02:
            case G03:
                file_content.append("G0" + QString::number(compact_data.at(i).Command) + " X" +
                           QString::number(compact_data.at(i).X) + " Y" + QString::number(compact_data.at(i).Y)
                           + " Z" + QString::number(compact_data.at(i).Z) +
                               + " I" +QString::number(compact_data.at(i).I) +" J" +QString::number(compact_data.at(i).J) +
                               " F" + QString::number(compact_data.at(i).Feed));
                file_content.append("\r");
            break;
        }
    }
    QFile file(path);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text)){
        file.write(file_content.toStdString().c_str(), file_content.length());
        file.close();
    }else{
        return ERROR_WRITE_FILE;
    }
    return GCODE_OK;
}
double Gcode_Decoder::adjust_gradiant(double raw_value)
{
    double p_raw_value = fabs(raw_value);

    if(p_raw_value > M_PI){
        return 2*M_PI - p_raw_value;
    }else{
        return p_raw_value;
    }
}
double Gcode_Decoder::solve_quad(double a, double b, double c)
{
    double denta = fabs(b*b - 4*a*c);
    double x1, x2;
    x1 = (-b+sqrt(denta))/(2*a);
    x2 = (-b-sqrt(denta))/(2*a);
    if(x1 < x2){
       return x1;
    }else{
       return x2;
    }
}
Gcode_Decoder *gcode = new Gcode_Decoder();
