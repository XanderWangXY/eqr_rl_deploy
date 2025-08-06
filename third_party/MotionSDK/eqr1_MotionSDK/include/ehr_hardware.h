#ifndef EHR_HARDWARE_H_
#define EHR_HARDWARE_H_

#include "ehr_hardware_interface.h"
#include "visibility.h"
#include <string>

class PUBLIC_API EhrHardware
{
public:
    EhrHardware();
    ~EhrHardware();
    EhrHardwareInterface* GetInterface();
private:
    EhrHardwareInterface* m_interface_;   
};
#endif // EHR_HARDWARE_H_
