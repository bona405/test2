#ifndef __SPIBEAM_CONSOLE_RUNNER_H__
#define __SPIBEAM_CONSOLE_RUNNER_H__

#include "Runner.h"
#include "SpiwriteCommand.h"

namespace SpiBeam {


class ConsoleRunner : public Runner
{
public:
    ConsoleRunner( TransportMap& transport_map, ArrayInfoMap& arraym );
    ~ConsoleRunner();
 
    virtual void Run() override;
    void Run(SpiwriteProtocol::MemoryWriter& writer);
    void SetMaxTransferSizeInBytes( int transfer_size );


private:
    struct Impl;
    friend class ConsoleRunner::Impl;
    SpiwriteProtocol::MemoryWriter writer;
    Impl *impl_;
    
};


}

#endif