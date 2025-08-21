#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>
#include "AimRunner.h"
#include "CodeGenerator.h"
#include "SpiwriteCommand.h"
#include "ArrayFactory.h"
#include "AntennaHandler.h"
#include "Timeval.hpp"
#include "UDPPoint.h"
#include "vk_log.h"


namespace SpiBeam {

using namespace Common;
using namespace AIM::AntennaProtocol;

struct AimRunner::Impl : AIM::AntennaProtocol::AntennaHandler
{
    AimRunner& owner;
    DestinatedUDPPoint remote;
    AimConfig cfg;
    AimRunner::OnReceivedMessageFn on_received_message_fn;
    
    Impl( AimRunner& consoler ) : owner( consoler )
    {
        on_received_message_fn = [](uint32_t msg){};
    }

    virtual void OnPreMessage( const Frame& f )
    {
        INFO_LOG( "%s", Timeval::Now().ToISO8601().c_str() );

        AntennaHandler::OnPreMessage( f );

        const char *msgName = "NA";
        try
        {
            msgName = ProtocolInfo::Instance().MessageName(f.Head.MessageType);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        INFO_LOG( "Received Sequence:%d, MessageType:0x%02x(%s), Ctrl:0x%02x", 
            f.Head.Sequence, f.Head.MessageType, msgName, f.Head.Ctrl );
    }

    virtual void OnPostMessage( const Frame& f ) 
    {
        on_received_message_fn( f.Head.MessageType ); 
    }

    virtual void OnMessage( const Header& head, const MessageConfigSet& msg)
    {
/*
typedef enum {
	CFG_CALIBRATE = 1,
	CFG_AZ_OFFSET,
	CFG_EL_OFFSET,
	CFG_INSTALL_AZ_OFFSET,
	CFG_INSTALL_EL_OFFSET,
	CFG_HOMING_DISABLE,
	MAX_CFG_CODE
} cfg_code_e;
*/
        INFO_LOG( "ConfigSet" );
        std::cout << "\r\nConfigSet";
        for( auto e : msg.Entries ) 
        {
            INFO_LOG( "code:%d, value:%d", e.code, e.value );
        }
    }

    virtual void OnMessage( const Header& head, const MessageTimeSync& msg)
    {

        INFO_LOG( "TimeSync - time:%s, flag:%d", 
            Timeval( msg.Time.tv).ToISO8601().c_str(), msg.Time.flag );
    }

    virtual void OnMessage( const Header& head, const MessageBlockageInfo& msg)
    {
        INFO_LOG( "Blockage - cmd:0x%02x", msg.Cmd );
        for( auto z : msg.Zones )
        {
            INFO_LOG( "az:[%d-%d], el[%d-%d]", z.az_start, z.az_end, z.el_start, z.el_end );
        }
    }

    virtual void OnMessage( const Header& head, const MessageTrack& track )
    {
        INFO_LOG( "Track - Type:0x%02x, TrackID:%d", track.Type, track.TrackID );

        int i = 0;
        for( auto e : track.Entries )
        {
            INFO_LOG( "[%d] - id:%d, az:%.2f, el:%.2f, time:%s", 
                i++, e.id, e.az/100.0, e.el/100.0, Timeval(e.tv).ToISO8601().c_str() );
        }
    } 

    virtual void OnMessage( const Header& head, const MessagePositionSummary& summy )
    {
        auto& s = summy.Content;
        Timeval tv( timeval{ (__time_t)s.seocond, (__suseconds_t)s.usec } );
        INFO_LOG( "nPositionSummary - track_id:%d, az[%.2f-%.2f], el[%.2f-%.2f], peak_az:%.2f, peak_el:%.2f, time:%s", 
            s.track_id,
            s.start_az/100.0, s.end_az/100.0, 
            s.start_el/100.0, s.end_el/100.0,
            s.peak_az/100.0, s.peak_el/100.0,
            tv.ToISO8601().c_str() );
    }    

    Runner::ArrayInfo& GetArrayInfoOrFirst( std::string name )
    {
        if( name.empty() )
        {
            return std::begin(owner.array_info_map_)->second;
        }

        if( auto I = owner.array_info_map_.find( name ); I != owner.array_info_map_.end())
        {
            return I->second;
        }
        throw std::runtime_error( Common::string_format( "init failed : no array with %s found\n", name.c_str() ));
    }

    void Run()
    {
        using namespace std;
        using namespace SpiBeam::Array;

        Controller::CodeGenerator cgen;
    }
};

AimRunner::AimRunner( TransportMap& transport_map, ArrayInfoMap& arraym, const AimConfig& cfg ) : 
    Runner(transport_map, arraym), 
    impl_(new Impl(*this)) 
{
    impl_->cfg = cfg;
    
    auto& up = impl_->remote;
    up.SetDestination( cfg.remote_ip.c_str(), cfg.remote_port);
    up.Bind( cfg.local_port, [this](const char*msg, int len, const sockaddr* sender) { 
        impl_->OnReceive( msg, len );
    });
    impl_->SetOnSend([&up](const char*frame, int len){ up.Send(frame,len); });
}

AimRunner::~AimRunner() 
{
    delete impl_;
}

void AimRunner::Run()
{
    impl_->Run();
}

void AimRunner::SetOnReceivedMessageFn( OnReceivedMessageFn fn )
{
    impl_->on_received_message_fn = fn;
}


}

