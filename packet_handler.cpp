#include "packet_handler.h"

packet_handler::packet_handler()
{

}

void packet_handler::categorize(std::vector<uint8_t> &packet)
{
    int number_of_state = strlen(end_header);
    int sync_state = 0;
    int distance = 0;
    int packet_size = packet.size();
    int count = 0;
    do{
        if(packet.at(count) == (uint8_t)(end_header[sync_state])){
            sync_state++;
        }else if(packet.at(count) == (uint8_t)(end_header[0])){
            sync_state = 1;
        }else{
            sync_state = 0;
        }
        distance++;
        count++;
        if(number_of_state == sync_state){
            QByteArray temper; // empty byte array
            for(int temp = 0; temp < distance; temp++){
                temper.push_back(packet.at(temp));
            }
//            for(auto val: packet){
//                temper.push_back(val);
//            }
            packet_extract(temper, count >= (packet_size));
            packet.erase(packet.begin(), packet.begin() + distance);
            distance = 0;
            sync_state = 0;
            count = 0;
            packet_size = packet.size();
        }
    }while(count < packet_size);
}

void packet_handler::packet_extract(QByteArray packet, bool end_buffer)
{
    if(packet.at(0) == 0x28){
        if(packet.at(1) == packet.count() - 2){
            archive_buffer.clear();
            routing(packet.mid(2, packet.at(1) - 2));
        }else{
            if(end_buffer == true){
                //packet length error
                log_console("Packet Length Error");
                return;
            }else{
                //move data to archive buffer, check another round for end flag
                archive_buffer.clear();
                archive_buffer.push_back(packet);
            }
        }
    }else{
        if(archive_buffer.size() != 0){
            archive_buffer.push_back(packet);
            if(archive_buffer.at(1) == archive_buffer.count() - 2){
                routing(archive_buffer.mid(2, archive_buffer.at(1) - 2));
            }else{
                //packet length error
                log_console("Packet Length Error");
                return;
            }
            archive_buffer.clear();
        }else{
            log_console("Header sync Error");
            //header error
            return;
        }
    }
}

void packet_handler::routing(QByteArray packet)
{
    int packet_length = packet.length();
    switch (packet.at(0)){ // Transmission protocol
        case FILE_TRANSMISION:

        break;
        case COMMAND_TRANSMISION:
            // error format
        break;
        case RESPONSE_TRANSMISION:
            switch (packet.at(1)) { // RPD type
                case RPD_IDLE:

                break;
                case RPD_BUSY:

                break;
                case RPD_POSITION:
                    if(packet.at(2) == CMD_READ_POSITION){ // check if CMD_ID is correct
                        Scara_position_received(packet.mid(3, packet_length-3));
                    }else{
                        log_console("Incorrect position packet format");
                        // incorrect packet format
                    }
                break;
                case RPD_START:
                case RPD_RUNNING:
                case RPD_DONE:
                case RPD_OK:
                case RPD_DUTY:
                case RPD_STOP:
                case RPD_ERROR:
                    Detail_Status_Handler(packet.mid(1, packet_length-1), DISPLAY_RPD_DETAIL);
                break;
                case RDP_GCODE_PROCESS:{
                    Detail_Status_Handler(packet.mid(1, packet_length-1), DISPLAY_GCODE_PROCESS);
                }
                break;
                case NUM_OF_RESPOND:

                break;
            }
        break;
        default:
        break;
    }
}

void packet_handler::Scara_position_received(QByteArray data)
{
    Scara_Position_RawData RawData;
    std::memcpy(&RawData, data.data(), data.size());
//    Scara_Position_RawData *RawData = reinterpret_cast<Scara_Position_RawData*>(data.data());
    Display_packet display_packet;
    display_packet.RealData.theta1 = (double)(RawData.raw_theta1*DATA_INVERSE_SCALE);
    display_packet.RealData.theta2 = (double)(RawData.raw_theta2*DATA_INVERSE_SCALE);
    display_packet.RealData.theta4 = (double)(RawData.raw_theta4*DATA_INVERSE_SCALE);
    display_packet.RealData.D3 = (double)(RawData.raw_D3*DATA_INVERSE_SCALE);
    Scara_Foward_Kinematic(display_packet);
//    display_packet.RealData.x = (double)(RawData->raw_x*DATA_INVERSE_SCALE);
//    display_packet.RealData.y = (double)(RawData->raw_y*DATA_INVERSE_SCALE);
//    display_packet.RealData.z = (double)(RawData->raw_z*DATA_INVERSE_SCALE);
//    display_packet.RealData.roll = (double)(RawData->raw_roll*DATA_INVERSE_SCALE);
    display_packet.action_id = DISPLAY_POSITION;
    emit on_display_event(display_packet);
}

void packet_handler::Detail_Status_Handler(QByteArray data, display_id id)
{
    Display_packet display_packet;
    display_packet.Respond_Type = (Robot_RespondTypedef)data.at(0);
    display_packet.action_id = id;
    switch (id) {
    case DISPLAY_RPD_DETAIL:{
        display_packet.Command_ID = (Robot_CommandTypedef)data.at(1);
        int packet_length = data.length();
        for(int i = 2; i < packet_length; i++){
            display_packet.Reference_String.append((Response_ID)data.at(i));
        }
    }break;
    case DISPLAY_GCODE_PROCESS:{
        display_packet.Command_ID = (Robot_CommandTypedef)data.at(1);
        display_packet.Contain_Data.append(data.at(2));
    }break;
    }

    emit on_display_event(display_packet);
}
void packet_handler::Scara_Foward_Kinematic(Display_packet &Scara_Coor)
{
    double a1 = 197.0;
    double a2 = 160.0;
    double a4 = 32.36;
    double d4 = 77.674;
    double d1 = 211.0;
    double Theta1 = Scara_Coor.RealData.theta1;
    double Theta2 = Scara_Coor.RealData.theta2;
    double Theta4 = Scara_Coor.RealData.theta4;
    double D3     = Scara_Coor.RealData.D3;
    Scara_Coor.RealData.x =   a1*cos(Theta1)
        + a2*cos(Theta1 + Theta2)
        + a4*cos(Theta1 + Theta2 - Theta4);
    Scara_Coor.RealData.y =   a1*sin(Theta1)
        + a2*sin(Theta1 + Theta2)
        + a4*sin(Theta1 + Theta2 - Theta4);
    Scara_Coor.RealData.z =   d1 - D3 - d4;
    Scara_Coor.RealData.roll = Theta1 + Theta2 - Theta4;

}
