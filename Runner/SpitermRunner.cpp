#include <string.h>
#include <algorithm>
#include "UDPPoint.h"
#include "SpitermRunner.h"
#include "SpiwriteProtocol.h"
#include "SpiwriteFrameHandler.h"
#include "LineParser.h"
#include "SpiwriteCommand.h"
#include "Instruction.h"

namespace SpiBeam {


class SpitermRunner::Impl : public SpiwriteProtocol::FrameHandler
{
public:
    Impl(SpitermRunner* owner) 
        : owner_(owner), spi_command_( owner_->transport_, &code_gen_, &parser_)
    {
        SetOnSend( [this](const uint8_t*buf, int len){ udp_point_.Send( (const char*)buf, len); } );
    }

public:
    UDPConfig udp_config_;
    Common::DestinatedUDPPoint udp_point_;
    Parser::LineParser parser_;
    SpitermRunner *owner_;
    Controller::CodeGenerator code_gen_;
    SpiwriteProtocol::SpiwriteCommand spi_command_;

    void OnMessage( const SpiwriteProtocol::Header& head, const SpiwriteProtocol::MessageLines& msg)
    {
        std::string rep = "";
        
        // string_view를 string으로 변환
        std::string full_message(msg.GetStringLines());
        
        // 바이너리 명령어인지 체크
        if (full_message.length() >= 7 && full_message.substr(0, 7) == "BINARY:") {
            // 바이너리 데이터는 라인 분할 없이 전체를 처리
            try {
                auto r = spi_command_.Execute(full_message);
                for(uint32_t v : r.responses) {
                    Controller::SpiReadback rb(v);
                    rep += Common::string_format("%04x[%d]\r\n", rb.Value(), rb.Length());
                }
                
                if(!r.message.empty()) {
                    rep += (r.message + "\r\n");
                }
            }
            catch(const std::exception& e) {
                rep += Common::string_format("Error : %s\r\n", e.what());
            }
        }
        else
        {
            for( const auto& line : parser_.SplitLines( msg.GetStringLines() ) )
            {
                try
                {
                    //auto r = spi_command_.Execute( parser_.Tokenize( line ) );
                    auto r = spi_command_.Execute( std::string(line) );
                    for( uint32_t v : r.responses ) 
                    {
                        Controller::SpiReadback rb( v );
                        rep += Common::string_format( "%04x[%d]\r\n", rb.Value(), rb.Length() );
                    }
    
                    if( !r.message.empty() )
                    {
                        rep += (r.message + "\r\n");
                    }
                }
                catch(const std::exception& e)
                {
                    rep += Common::string_format( "Error : %s\r\n", e.what() );
                }
            }
        }

        rep += "sch_VAIC> ";

        using namespace SpiwriteProtocol;

        MessageLines rmsg( rep.c_str() );

        Send( Frame{ 
            Header{ 
                MSG_STRAT_CODE, GetSequenceAndIncrement(), MSG_LINES, (uint32_t)rmsg.lines.size()
            }.ToNetwork(),
            MessageRaw( std::move(rmsg.lines))
        });
    }
};

SpitermRunner::SpitermRunner( TransportMap& transport_map, ArrayInfoMap& arraym, const UDPConfig& cfg ) : 
    Runner(transport_map, arraym)
    , impl_( new Impl(this) ) 
{
    impl_->udp_config_ = cfg;



    auto& up = impl_->udp_point_;
    up.SetDestination( cfg.remote_ip.c_str(), cfg.remote_port);
    up.Bind( cfg.local_port, [this](const char*msg, int len, const sockaddr* sender) { 
        this->impl_->OnReceive( (const uint8_t*) msg, len );
    });
}

SpitermRunner::~SpitermRunner() 
{
    delete impl_;
}


void SpitermRunner::Run()
{
    // ...    
}

}
