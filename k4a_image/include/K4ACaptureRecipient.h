#ifndef K4A_CAPTURE_RECIPIENT_H
#define K4A_CAPTURE_RECIPIENT_H

#include <k4a/k4a.h>

class K4ACaptureRecipient {
public:
    virtual void receiveFrame(k4a_capture_t capture) = 0;
};

#endif