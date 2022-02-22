#ifndef _UDPLOGGER_H_
#define _UDPLOGGER_H_

#include <stdint.h>
#include <cstddef>
#include "configuration.h"
#include "Print.h"


class UdpLogger : public Print {
    public:
        virtual size_t write(uint8_t);
        virtual size_t write(const uint8_t *buffer, size_t size);
};

extern UdpLogger & logger;


#endif