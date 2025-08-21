#ifndef __SPIBEAM_SPITERM_RUNNER_H__
#define __SPIBEAM_SPITERM_RUNNER_H__

#include <string>
#include "Runner.h"

namespace SpiBeam {

struct UDPConfig
{
    int local_port;
    std::string remote_ip;
    int remote_port;
};

class SpitermRunner : public Runner
{
public:
    SpitermRunner( TransportMap& transport_map, ArrayInfoMap& arraym, const UDPConfig& cfg );
    ~SpitermRunner();

    void Run();

private:
    struct Impl;
    Impl *impl_;
};


}

#endif