#include <string.h>
#include "SpiwriteProtocol.h"

namespace SpiBeam {
namespace SpiwriteProtocol {


class FrameHandler
{
public:
    using SendFn = std::function<void(const uint8_t*, int)>;
    void SetOnSend( SendFn fn ) { on_send_ = fn; }

    void OnReceive(const uint8_t *packet, int len)
    {
        for(int processed = 0; processed < len; )
        {
            auto decoded = DecodeFrame( packet+processed, len-processed );
            processed += decoded.Length();

            if ( decoded.head.start != SpiwriteProtocol::MSG_STRAT_CODE) {
                continue;
            };

            OnPreMessage( decoded );
            if( decoded.head.message_type == MSG_LINES)
            {
                OnMessage( decoded.head, MessageLines( std::move(decoded.message) ) );
            }
        }        
    }

    void Send( const SpiwriteProtocol::Frame& frame )
    {
        auto data = frame.DeepCopy();
        on_send_( &data[0], data.size() );
    }

    void Ack( uint32_t sequence, uint32_t msg_type = SpiwriteProtocol::MSG_ACK )
    {
        Send( SpiwriteProtocol::Frame {
            SpiwriteProtocol::Header {
                SpiwriteProtocol::MSG_STRAT_CODE,
                sequence,
                msg_type,
                0 }.ToNetwork()
            , SpiwriteProtocol::MessageRaw()
        } );
    }
 
    virtual void OnPreMessage( const Frame& f ) 
    {
        if ( f.head.message_type == MSG_ACK ) return;
           
        Ack( f.head.sequence, SpiwriteProtocol::MSG_ACK );
    }
    
    virtual void OnMessage( const Header& head, const MessageLines& msg)
    {  
    }

    uint32_t GetSequenceAndIncrement() 
    {
        return sequence_++;
    }

private:
    SendFn on_send_;
    uint32_t sequence_ { 0 };

};



}
}
