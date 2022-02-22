

#include "udplogger.h"
#include "WiFiUdp.h"

class UdpLoggerImpl : public UdpLogger {

private:
    bool init = false;
    WiFiUDP udp;

public:

    virtual size_t write(uint8_t){
        return 1;
    }

    virtual size_t write(const uint8_t *buffer, size_t size)
    {
        if( !init )
        {
            if( ! udp.begin(60123) )
                return size;
            init=true;
        }
        IPAddress ip(192,168,1,11);
        udp.beginPacket(ip,62731);
        udp.write(buffer,size);
        udp.endPacket();
        delay(100);
        return size;
    }

};

UdpLoggerImpl loggerImpl;
UdpLogger & logger = loggerImpl;

