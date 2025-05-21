#ifndef EHR_HARDWARE_INTERFACE_H_
#define EHR_HARDWARE_INTERFACE_H_

#include "ehr_hardware_def.h"
#include "visibility.h"
#include <string>
class PUBLIC_API EhrHardwareInterface{
public:
    virtual ~EhrHardwareInterface() {};
    virtual bool Init(const char* data, int len) = 0;
    virtual bool Read(ehr_body_state *state) = 0;
    virtual bool Write(ehr_body_state &state) = 0;
    virtual bool Deinit() = 0;
};
#endif // EHR_HARDWARE_INTERFACE_H_

