#ifndef __SPIBEAM_AIM_RUNNER_H__
#define __SPIBEAM_AIM_RUNNER_H__

#include <functional>
#include "Runner.h"

namespace SpiBeam {

struct AimConfig
{
    int local_port;
    std::string remote_ip;
    int remote_port;
};


class AimRunner : public Runner
{
public:
    AimRunner( TransportMap& transport_map, ArrayInfoMap& arraym, const AimConfig& cfg );
    ~AimRunner();
 
    void Run();

    using OnReceivedMessageFn = std::function<void(uint32_t)>;
    void SetOnReceivedMessageFn( OnReceivedMessageFn fn );

private:
    struct Impl;
    Impl *impl_;
};


}

#endif