#ifndef PACKET_HANDLER_H
#define PACKET_HANDLER_H

#include <QObject>
#include <QtCore>
#include "define_parameter.h"
#define RECEIVE_END "})"
class packet_handler: public QObject
{
    Q_OBJECT
public:
    packet_handler();
    define_parameter *system_parameter;
    void categorize(std::vector<uint8_t> &packet);
    int32_t number_of_packet;
signals:
    void on_display_event(Display_packet);
private:
    void routing(QByteArray packet);
    void packet_extract(QByteArray packet, bool end_buffer);
    void Scara_position_received(QByteArray data);
    void Detail_Status_Handler(QByteArray data, display_id id);
    void Scara_Foward_Kinematic(Display_packet &Scara_Coor);
    QByteArray archive_buffer;
    bool archive_status = false;
    const char* end_header = RECEIVE_END;
};

#endif // PACKET_HANDLER_H
