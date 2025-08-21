#include "string_util.hpp"
#include "SpiwriteProtocol.h"

namespace SpiBeam {
namespace SpiwriteProtocol {

bool MessageRaw::s_ShowCopyConstructorMessage = true;

Frame DecodeFrame( const uint8_t* frame, int len )
{
    if ( len < sizeof(Header) )
        throw std::logic_error( "Frame is not ready yet!" );

    Header head = Header::FromNetwork( frame );
    
    // 디버깅 출력 추가
    // printf("DEBUG: len=%d, sizeof(head)=%zu, head.message_length=%u\n", 
    //     len, sizeof(head), head.message_length);

    const uint8_t *msg = frame + sizeof(head);
    if ( sizeof(head)+ head.message_length > len )
        throw std::logic_error( Common::string_format( 
            "%d recevied but %d needed", len,sizeof(head)+ head.message_length ));

    return Frame { 
        head, 
        std::vector<uint8_t>(msg, msg + head.message_length)
    };
}


}
}
