#ifndef __SPIBEAM_RUNNER_H__
#define __SPIBEAM_RUNNER_H__

#include <string>
#include "Transport.h"
#include "ArrayBase.h"
#include "CalExecutor.h"

namespace SpiBeam {

class Runner
{
public:
    struct ArrayInfo
    {
        Array::ArrayBase& array;
        Controller::Transport& transport;
        std::shared_ptr<Array::Calibration::CalExecutor> cal_executor;
    };

    using ArrayInfoMap = std::map<std::string,ArrayInfo>;
    using TransportMap = std::map<std::string,Controller::Transport&>;
    
    Runner( TransportMap& transport_map, ArrayInfoMap& array_info_map ) 
        : transport_map_(transport_map), array_info_map_( array_info_map ), transport_( transport_map_.begin()->second ){}

    virtual ~Runner() {}

    virtual void Run() = 0;
    virtual void Join() {};


protected:
    TransportMap& transport_map_;
    ArrayInfoMap& array_info_map_;
    Controller::Transport& transport_;
};


}



#endif