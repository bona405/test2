#ifndef __SPIBEAM_SPIWRITE_PROTOCOL_H__
#define __SPIBEAM_SPIWRITE_PROTOCOL_H__

#include <string.h>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <string_view>
#include <netinet/in.h>

namespace SpiBeam {
namespace SpiwriteProtocol {

enum {
    MSG_STRAT_CODE  = 0x1077E110,

	MSG_ACK         = 0x00000001,
	MSG_LINES       = 0x00000002,	
};

struct __attribute__ ((packed)) Header
{
    uint32_t start = 0;
    uint32_t sequence = 0;
    uint32_t message_type = 0;
    uint32_t message_length = 0;

    static Header FromNetwork( const uint8_t *raw )
    {
        const Header& nh = *(const Header*)raw;
        return Header { 
            ntohl( nh.start ),
            ntohl( nh.sequence ),
            ntohl( nh.message_type ),
            ntohl( nh.message_length ),
        };
    }

    Header ToNetwork()
    {
        return Header { 
            htonl( start ),
            htonl( sequence ),
            htonl( message_type ),
            htonl( message_length ),
        };
    }    
};  

struct MessageBase {};

struct MessageRaw : MessageBase
{
    MessageRaw() {}
    MessageRaw( std::vector<uint8_t>&& _data ) : data( std::move(_data)) {}
    MessageRaw( MessageRaw&& raw ) : data( std::move( raw.data ) ) {}

    MessageRaw( const MessageRaw& raw ) : data( raw.data ) 
    {
        if( s_ShowCopyConstructorMessage )
        {
            std::cout << "Better not copy !\r\n"; 
        }
    }

    static void ShowCopyConstructorMessage( bool onoff )
    {
        s_ShowCopyConstructorMessage = onoff;
    }

    static bool s_ShowCopyConstructorMessage; 

    const uint8_t* GetData() const { return &data[0];} 

    std::vector<uint8_t> data; 
};

struct Frame
{   
    Header head;
    MessageRaw message;

    int Length() const { return (int)( sizeof(head) + message.data.size()); }

    std::vector<uint8_t> DeepCopy() const 
    {   
        std::vector<uint8_t> data( Length() );
        memcpy( &data[0], &head, sizeof(head));
        memcpy( &data[sizeof(head)], &message.data[0], message.data.size() );
        return data;
    }
};

Frame DecodeFrame( const uint8_t* frame, int len );

struct MessageLines : MessageBase
{
    MessageLines( MessageRaw&& msg_raw ) : lines( std::move(msg_raw.data) ) {} 
    MessageLines( std::vector<uint8_t>&& raw ) : lines( std::move(raw) ) {}

    static MessageLines DeepCopy( const MessageRaw& raw ) { return MessageLines( raw.data ); }
    
    MessageLines( const char *str_lines ) 
    { 
        auto len = strnlen( str_lines, 1400 );
        if( len >= 1400 ) throw std::logic_error( "too long string !!!");

        lines.resize( len+1 );
        lines[len] = '\0';
        std::copy( str_lines, str_lines+len, (char*)(&lines[0]) );
    }

    std::string_view GetStringLines() const { return std::string_view( (const char*)&lines[0], lines.size()-1 ); }

    std::vector<uint8_t> lines; 

private:
    MessageLines( const std::vector<uint8_t>& data ) : lines( data.begin(), data.end() ) {} 

};



}
}


#endif
