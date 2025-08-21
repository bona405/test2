#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>
#include "ConsoleRunner.h"
#include "LineParser.h"
#include "CodeGenerator.h"
#include "SpiwriteCommand.h"
#include "ArrayFactory.h"
#include "JsonHelper.hpp"

#include <cstdio>
#include <vector>
#include <cmath>
#include <tuple>
#include <iomanip>
#include <set>

constexpr uintptr_t BASE_ADDR = 0x43C00000;
constexpr uintptr_t FIFO_ADDR = 0x43C40000;

namespace SpiBeam {

using namespace std;

// 전역 변수들 초기화
float az_value = 0.0f;
float el_value = 0.0f;
long long unsigned freq_Hz = 1000000000ULL;  // 기본값 1GHz

constexpr float INTELLIAN_PI = 3.14159265359f;  // 더 정확한 PI 값

inline float to_radian(float deg)
{
    return deg * INTELLIAN_PI / 180.0f;
}

inline float to_degree(float rad)
{
    return rad * 180.0f / INTELLIAN_PI;
}

inline float normalize_degrees(float degrees)
{
    if(degrees >= 0.0 && degrees < 360.0)
    {
        return degrees;
    }

    float norm = std::fmod(degrees, 360.0);
    if(norm < 0)
    {
        norm += 360.0;
    }

    return norm;
}

float phase(float xi, float yi) 
{
    // 입력값 검증
    if(isnan(az_value) || isnan(el_value) || freq_Hz == 0) {
        cout << "Error: Invalid input values - az:" << az_value 
             << " el:" << el_value << " freq:" << freq_Hz << endl;
        return 0.0f;
    }

    float impl_phi = - az_value;
    float phi_rad = to_radian(impl_phi);

    float c_theta = std::cos(to_radian(el_value));
    float c_phi = std::cos(phi_rad);
    float s_phi = std::sin(phi_rad);

    const float SPEED_OF_LIGHT = 300000000;
    float lambda = SPEED_OF_LIGHT / freq_Hz;
    float k0 = -2.0f * INTELLIAN_PI / lambda / 1000;

    float p = to_degree(k0 * (xi * c_theta * c_phi + yi * c_theta * s_phi));
    float p_nor = normalize_degrees(p);

    // 각 변수들을 출력
    // std::cout << "===============" << std::endl;
    // std::cout << "k0 = " << k0 << std::endl;
    // std::cout << "xi = " << xi << std::endl;
    // std::cout << "yi = " << yi << std::endl;
    // std::cout << "c_theta = " << c_theta << std::endl;
    // std::cout << "c_phi = " << c_phi << std::endl;
    // std::cout << "s_phi = " << s_phi << std::endl;

    // std::cout << "p =  " << p << std::endl;
    // std::cout << "p_nor  = " << p_nor << std::endl;
    // std::cout << "===============" << std::endl;

    return p_nor;
}


struct ConsoleRunner::Impl
{
    ConsoleRunner& owner;
    int transfer_size_in_bytes = 32;
    Controller::Transport* current_transport;

    Impl( ConsoleRunner& consoler ) : owner( consoler )
    {
        current_transport = &owner.array_info_map_.begin()->second.transport; 
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


    int cnt77 = 1;


    void Execute( Runner::ArrayInfo& arrayinfo, std::vector<Controller::Code> codes)
    {
        using namespace std::chrono;

        int read_count = 0;
        Controller::Transport& array_transport = arrayinfo.transport;

        std::vector<uint32_t> buf;
        buf.reserve( transfer_size_in_bytes );
        for( auto& code : codes )
        {
            auto words = code.CopyWords();
            if( (buf.size() + words.size() * sizeof(uint32_t)) > transfer_size_in_bytes )
            {
                array_transport.Write( buf, 0 );
                buf.clear();                
            }

            std::copy( words.begin(), words.end(), std::back_inserter(buf) );
            read_count += code.GetReadCount();
        }

        if( buf.size() > 0 )
        {
            array_transport.Write( buf, 0 );
            buf.clear();
        }

        if( read_count > 0 )
        {
            while( array_transport.ReceviedCount( 0 ) < read_count )
            {
                std::this_thread::sleep_for( 1ms );
            }

            auto reads = array_transport.Read( read_count, 0 );
            std::vector<Controller::SpiReadback> readbacks;
            readbacks.reserve( reads.size());
            std::transform( reads.begin(), reads.end(), std::back_inserter(readbacks), 
                [](uint32_t r){ return Controller::SpiReadback(r); });

            arrayinfo.array.Readback( readbacks );
        }
    }

    void Print( Array::ArrayBase& array, std::string name )
    {
        if( name == "amplitude" || name == "phase")
        {
            auto& m = array.GetStatus().Get<Math::MatFloat>( name );
            m.Dump( "%4.1f ");
        }
        else
        {
            auto& m = array.GetStatus().Get<Math::MatInt>( name );
            m.Dump( "%3d ");
        }
    }

    void Beam( Array::ArrayBase& array, float az, float el )
    {
        if( array.GetCfg().port.poles.Size() == 0 )
        {
            auto m = array.GetLayoutFormer().FormPhase( az, el );
            array.GetStatus().GetPhase() = m;
        } 
        else
        {
            auto m = array.GetLayoutFormer().FormCircularPhases( az, el );
            array.GetStatus().GetPhase() = m;
        }
    }

    void Run() 
    {
        owner.Run(owner.writer);
    }

    void Run(SpiwriteProtocol::MemoryWriter& writer)
    {
        using namespace std;
        using namespace SpiBeam::Array;

        Parser::LineParser parser;
        Controller::CodeGenerator cgen;

        float dx = 5.0f;
        float dy = 5.0f;

        int is_tx = 0;

        std::map<int, std::deque<uint8_t>> byte_queues;
        std::map<int, std::mutex> queue_mutexes;

        uintptr_t base_address;
        static int count__ = 1;

        auto change_bus_process = [&](int spi_id) -> void
        {
            printf(";spi_id => %d\n", spi_id);


            // 현재 bus 종료 처리
            writer.writeMemory(base_address + 0x0014, 0x280);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            printf("sendln \"devmem 0x%08x 32 0x280\"\n", base_address + 0x0014);
            printf("mpause 10\n");

            // 인터럽트 상태 확인
            uint32_t interrupt_value;
            writer.readMemory(base_address, interrupt_value);
            printf(";base_address => 0X%08x interrupt_value => 0x%08x\n", base_address, interrupt_value);
            printf("mpause 10\n"); 

            // 인터럽트 초기화
            writer.writeMemory(base_address, 0xffffffff);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            printf("sendln \"devmem 0x%08x 32 0xffffffff\"\n", base_address);
            printf("mpause 10\n");

            // 남은 FIFO DATA SIZE 확인
            uint32_t remaining_fifo_data_size;
            writer.readMemory(base_address + 0xC, remaining_fifo_data_size);
            printf(";remaining_fifo_data_size => 0x%08x\n", remaining_fifo_data_size);
            printf("mpause 10\n");


            // 새로운 bus 시작 처리
            if(spi_id == 7)
            {


            }
            else
            {
                base_address = 0x43c40000 + (spi_id * 0x10000);
                uint32_t init_value_ = 0x2;
                writer.writeMemory(base_address + 0x2C, init_value_);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01X\"\n", base_address + 0x2C, init_value_);
                printf("mpause 10\n");
            }
        };

        for(;;)
        {
            // 1단계: tx 또는 rx 입력 받기
            cout << "Enter tx or rx > ";
            string txrx_input;
            getline(std::cin, txrx_input);
            
            // 빈 입력 처리
            if(txrx_input.empty()) continue;
            
            // tx/rx 처리
            if(txrx_input == "tx")
            {
                dx = 5.0f;
                dy = 5.0f;
                freq_Hz = 29500000000ULL;
                is_tx = 1;

                #if 1
                printf("\n@@@ tx 패널 초기화 시작 @@@\n");
            
                // VAIC RESET OFF
                uintptr_t vaic_reset_addr = 0x43c28004;
                uint32_t reset_off = 0xff;
                writer.writeMemory(vaic_reset_addr, reset_off);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", vaic_reset_addr, reset_off);
                printf("mpause 10\n");
            
                // VAIC RESET ON
                uint32_t reset_on = 0x0;
                writer.writeMemory(vaic_reset_addr, reset_on);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", vaic_reset_addr, reset_on);
                printf("mpause 10\n");
            
                // FIFO init
                for (int i = 0; i < 8; i++) 
                {
                    uintptr_t fifo_addr = FIFO_ADDR + (i * 0x10000);
            
                    // 인터럽트 상태 확인
                    uint32_t interrupt_value;
                    writer.readMemory(fifo_addr, interrupt_value);
                    printf(";interrupt check addr -> 0x%08x, value -> 0x%08x\n", fifo_addr, interrupt_value);
                    printf("mpause 10\n");
            
                    // while (true)
                    // {
                    //     uint32_t interrupt_value;
                    //     if (!writer.readMemory(fifo_addr, interrupt_value)) {
                    //         printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(fifo_addr));
                    //         break;
                    //     }
                    
                    //     if (interrupt_value == 0) 
                    //     {
                    //         printf(";interrupt check ok !\n");
                    //         break;
                    //     }
            
                    //     printf("mpause 100\n");
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    // }
            
                    // 인터럽트 클리어
                    uint32_t interrup_clear_value = 0xffffffff;
                    writer.writeMemory(fifo_addr, interrup_clear_value);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_addr, interrup_clear_value);
                    printf("mpause 10\n");
                }
            
                for(int i = 0; i < 8; i++)
                {
                    // start address
                    uintptr_t start_address_ = FIFO_ADDR + (i * 0x10000) + 0x2C;
                    uint32_t start_address_value__ = 0x2;
                    writer.writeMemory(start_address_, start_address_value__);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", start_address_, start_address_value__);
                    printf("mpause 10\n");
            
            
                    uintptr_t fifo_1_address_ = FIFO_ADDR + (i * 0x10000) + 0x10;
                    uint32_t broadcast_value1 = 0x60000000;
                    writer.writeMemory(fifo_1_address_, broadcast_value1);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value1);
                    printf("mpause 10\n");
                    uint32_t broadcast_value2 = 0x60010688;
                    writer.writeMemory(fifo_1_address_, broadcast_value2);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value2);
                    printf("mpause 10\n");
            
                    ////////////////////////////////////
                    // 0x25, 3D, 45, 5D
                    ////////////////////////////////////
                    uint32_t broadcast_value3 = 0x6025A91A;
                    writer.writeMemory(fifo_1_address_, 0x6025A91A);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value3);
                    printf("mpause 10\n");
                    uint32_t broadcast_value4 = 0x603DA91A;
                    writer.writeMemory(fifo_1_address_, 0x603DA91A);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value4);
                    printf("mpause 10\n");
                    uint32_t broadcast_value5 = 0x6045A91A;
                    writer.writeMemory(fifo_1_address_, 0x6045A91A);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value5);
                    printf("mpause 10\n");
                    uint32_t broadcast_value6 = 0x605DA91A;
                    writer.writeMemory(fifo_1_address_, 0x605DA91A);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value6);
                    printf("mpause 10\n");
            
                    ////////////////////////////////////
                    // 0x26, 3E, 46, 5E
                    ////////////////////////////////////
                    uint32_t broadcast_value7 = 0x60260E7F;
                    writer.writeMemory(fifo_1_address_, 0x60260E7F);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value7);
                    printf("mpause 10\n");
                    uint32_t broadcast_value8 = 0x603E0E7F;
                    writer.writeMemory(fifo_1_address_, 0x603E0E7F);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value8);
                    printf("mpause 10\n");
                    uint32_t broadcast_value9 = 0x60460E7F;
                    writer.writeMemory(fifo_1_address_, 0x60460E7F);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value9);
                    printf("mpause 10\n");
                    uint32_t broadcast_value10 = 0x605E0E7F;
                    writer.writeMemory(fifo_1_address_, 0x605E0E7F);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value10);
                    printf("mpause 10\n");
            
                    ////////////////////////////////////
                    // 0x27, 0x3F, 0x47,  0x5F
                    ////////////////////////////////////
                    uint32_t broadcast_value11 = 0x602703FE;
                    writer.writeMemory(fifo_1_address_, 0x602703FE);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value11);
                    printf("mpause 10\n");
                    uint32_t broadcast_value12 = 0x603F03FE;
                    writer.writeMemory(fifo_1_address_, 0x603F03FE);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value12);
                    printf("mpause 10\n");
                    uint32_t broadcast_value13 = 0x604703FE;
                    writer.writeMemory(fifo_1_address_, 0x604703FE);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value13);
                    printf("mpause 10\n");
                    uint32_t broadcast_value14 = 0x605F03FE;
                    writer.writeMemory(fifo_1_address_, 0x605F03FE);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value14);
                    printf("mpause 10\n");
            
                    // length : 56 Byte (0x38)
                    uintptr_t fifo_send_length = FIFO_ADDR + (i * 0x10000) + 0x14;
                    uint32_t fifo_send_length____ = 0x38;
                    writer.writeMemory(fifo_send_length, fifo_send_length____);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_send_length, fifo_send_length____);
                    printf("mpause 10\n");
            
                    // 인터럽트 상태 확인
                    uintptr_t intrerrupt_addr___ = FIFO_ADDR + (i * 0x10000);
                    uint32_t interrupt_value;
                    writer.readMemory(intrerrupt_addr___, interrupt_value);
                    printf(";interrupt check addr -> 0x%08x, value -> 0x%08x\n", intrerrupt_addr___, interrupt_value);
                    printf("mpause 10\n");
            
                    // while (true)
                    // {
                    //     uintptr_t intrerrupt_addr = 0x43c40000;
                    //     uint32_t interrupt_value;
                    //     if (!writer.readMemory(intrerrupt_addr, interrupt_value)) {
                    //         printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(intrerrupt_addr));
                    //         break;
                    //     }
                    
                    //     if (interrupt_value == 0) 
                    //     {
                    //         printf(";interrupt check ok !\n");
                    //         break;
                    //     }
                    
                    //     printf("mpause 10\n");
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    // }
            
                    // 인터럽트 클리어
                    uintptr_t intrerrupt_addr_ = FIFO_ADDR + (i * 0x10000);
                    uint32_t interrup_clear_value = 0xffffffff;
                    writer.writeMemory(intrerrupt_addr_, interrup_clear_value);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", intrerrupt_addr_, interrup_clear_value);
                    printf("mpause 10\n");
            
            
                }
            
            
                // Send Length
                uintptr_t length_addr = 0x43c00018; 
                uint32_t length_value = 0x4;
                writer.writeMemory(length_addr, length_value);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", length_addr, length_value);
                printf("mpause 10\n");
            
                // FIFO Execute : 0x1
                uintptr_t addr_1c = 0x43c0001c;
                uint32_t value_1c = 0x1;
                writer.writeMemory(addr_1c, value_1c);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", addr_1c, value_1c);
                printf("mpause 10\n");
            
                // FIFO 1~8 SEND : 0x1
                uintptr_t send_addr = 0x43c00014;
                uint32_t send_value = 0xff;
                writer.writeMemory(send_addr, send_value);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", send_addr, send_value);
                printf("mpause 10\n");
            
            
                // SEND FIFO CHECK !!!
                while (true)
                {
                    uintptr_t fifo_send_check_address = 0x43c00014;
                    uint32_t fifo_send_check_value;
                    if (!writer.readMemory(fifo_send_check_address, fifo_send_check_value)) {
                        printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(fifo_send_check_address));
                        printf("mpause 10\n");
                        break;
                    }
                
                    if (fifo_send_check_value == 0x0) 
                    {
                        printf(";ok. fifo send completed\n");
                        break;
                    }
                
                    printf("mpause 100\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            
            
                // 남은 FIFO DATA SIZE 확인
                uintptr_t remaining_fifo_address = 0x43c50000 + 0xc;
                uint32_t remaining_fifo_data_size;
                if (!writer.readMemory(remaining_fifo_address, remaining_fifo_data_size)) {
                    printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(remaining_fifo_address + 0xC));
                    printf("mpause 10\n");
                }
                else
                {
                    printf("@@@ remaining size => 0x%08x @@@\n", remaining_fifo_data_size);
                }
                
                remaining_fifo_address = 0x43c40000 + 0xc;
                if (!writer.readMemory(remaining_fifo_address, remaining_fifo_data_size)) {
                    printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(remaining_fifo_address + 0xC));
                    printf("mpause 10\n");
                }
                else
                {
                    printf("@@@ remaining size => 0x%08x @@@\n", remaining_fifo_data_size);
                }
            
            
                printf("@@@ tx 패널 초기화 끝 @@@\n");
            #endif
                
            }
            else if(txrx_input == "rx")
            {
                dx = 7.5f;
                dy = 7.5f;
                freq_Hz = 19700000000ULL;
                is_tx = 0;

                #if 1
                printf("rx 패널 초기화 시작 !!!!\n");
            
                // VAIC RESET OFF
                uintptr_t vaic_reset_addr = 0x43c28004;
                uint32_t reset_off = 0xff;
                writer.writeMemory(vaic_reset_addr, reset_off);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", vaic_reset_addr, reset_off);
                printf("mpause 10\n");
            
                // VAIC RESET ON
                uint32_t reset_on = 0x0;
                writer.writeMemory(vaic_reset_addr, reset_on);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", vaic_reset_addr, reset_on);
                printf("mpause 10\n");
            
                // FIFO init
                for (int i = 0; i < 8; i++) 
                {
                    uintptr_t fifo_addr = FIFO_ADDR + (i * 0x10000);
            
                    // 인터럽트 상태 확인
                    uint32_t interrupt_value;
                    writer.readMemory(fifo_addr, interrupt_value);
                    printf(";interrupt check addr -> 0x%08x, value -> 0x%08x\n", fifo_addr, interrupt_value);
                    printf("mpause 10\n");
            
                    // while (true)
                    // {
                    //     uint32_t interrupt_value;
                    //     if (!writer.readMemory(fifo_addr, interrupt_value)) {
                    //         printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(fifo_addr));
                    //         break;
                    //     }
                    
                    //     if (interrupt_value == 0) 
                    //     {
                    //         printf(";interrupt check ok !\n");
                    //         break;
                    //     }
            
                    //     printf("mpause 100\n");
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    // }
            
                    // 인터럽트 클리어
                    uint32_t interrup_clear_value = 0xffffffff;
                    writer.writeMemory(fifo_addr, interrup_clear_value);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_addr, interrup_clear_value);
                    printf("mpause 10\n");
                }
            
            
                for(int j = 0; j < 8; j++)
                {
                    // start address
                    uintptr_t start_address_ = FIFO_ADDR + (j * 0x10000) + 0x2C;
                    uint32_t start_address_value__ = 0x2;
                    writer.writeMemory(start_address_, start_address_value__);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", start_address_, start_address_value__);
                    printf("mpause 10\n");
            
            
                    uintptr_t fifo_1_address_ = FIFO_ADDR + (j * 0x10000) + 0x10;
                    uint32_t broadcast_value1 = 0x60000000;
                    writer.writeMemory(fifo_1_address_, broadcast_value1);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value1);
                    printf("mpause 10\n");
                    uint32_t broadcast_value2 = 0x6001068A;
                    writer.writeMemory(fifo_1_address_, broadcast_value2);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value2);
                    printf("mpause 10\n");
            
                    ////////////////////////////////////
                    // 0x25, 3D, 45, 5D
                    ////////////////////////////////////
                    uint32_t broadcast_value3 = 0x60206CDB;
                    writer.writeMemory(fifo_1_address_, broadcast_value3);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value3);
                    printf("mpause 10\n");
                    uint32_t broadcast_value4 = 0x60386CDB;
                    writer.writeMemory(fifo_1_address_, broadcast_value4);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value4);
                    printf("mpause 10\n");
                    uint32_t broadcast_value5 = 0x60406CDB;
                    writer.writeMemory(fifo_1_address_, broadcast_value5);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value5);
                    printf("mpause 10\n");
                    uint32_t broadcast_value6 = 0x60586CDB;
                    writer.writeMemory(fifo_1_address_, broadcast_value6);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value6);
                    printf("mpause 10\n");
            
                    ////////////////////////////////////
                    // 0x26, 3E, 46, 5E
                    ////////////////////////////////////
                    uint32_t broadcast_value7 = 0x60212FFF;
                    writer.writeMemory(fifo_1_address_, broadcast_value7);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value7);
                    printf("mpause 10\n");
                    uint32_t broadcast_value8 = 0x60392FFF;
                    writer.writeMemory(fifo_1_address_, broadcast_value8);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value8);
                    printf("mpause 10\n");
                    uint32_t broadcast_value9 = 0x60412FFF;
                    writer.writeMemory(fifo_1_address_, broadcast_value9);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value9);
                    printf("mpause 10\n");
                    uint32_t broadcast_value10 = 0x60592FFF;
                    writer.writeMemory(fifo_1_address_, broadcast_value10);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value10);
                    printf("mpause 10\n");
            
                    ////////////////////////////////////
                    // 0x27, 0x3F, 0x47,  0x5F
                    ////////////////////////////////////
                    uint32_t broadcast_value11 = 0x602203F8;
                    writer.writeMemory(fifo_1_address_, broadcast_value11);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value11);
                    printf("mpause 10\n");
                    uint32_t broadcast_value12 = 0x603A03F8;
                    writer.writeMemory(fifo_1_address_, broadcast_value12);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value12);
                    printf("mpause 10\n");
                    uint32_t broadcast_value13 = 0x604203F8;
                    writer.writeMemory(fifo_1_address_, broadcast_value13);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value13);
                    printf("mpause 10\n");
                    uint32_t broadcast_value14 = 0x605A03F8;
                    writer.writeMemory(fifo_1_address_, broadcast_value14);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value14);
                    printf("mpause 10\n");
            
                    // length : 56 Byte (0x38)
                    uintptr_t fifo_send_length = FIFO_ADDR + (j * 0x10000) + 0x14;
                    uint32_t fifo_send_length____ = 0x38;
                    writer.writeMemory(fifo_send_length, fifo_send_length____);
                    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_send_length, fifo_send_length____);
                    printf("mpause 10\n");
            
                    // 인터럽트 상태 확인
                    uintptr_t intrerrupt_addr___ = FIFO_ADDR + (j * 0x10000);
                    uint32_t interrupt_value;
                    writer.readMemory(intrerrupt_addr___, interrupt_value);
                    printf(";interrupt check addr -> 0x%08x, value -> 0x%08x\n", intrerrupt_addr___, interrupt_value);
                    printf("mpause 10\n");
            
                    // while (true)
                    // {
                    //     uintptr_t intrerrupt_addr = 0x43c40000;
                    //     uint32_t interrupt_value;
                    //     if (!writer.readMemory(intrerrupt_addr, interrupt_value)) {
                    //         printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(intrerrupt_addr));
                    //         break;
                    //     }
                    
                    //     if (interrupt_value == 0) 
                    //     {
                    //         printf(";interrupt check ok !\n");
                    //         break;
                    //     }
                    
                    //     printf("mpause 10\n");
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    // }
            
                    // 인터럽트 클리어
                    uintptr_t intrerrupt_addr_ = FIFO_ADDR + (j * 0x10000);
                    uint32_t interrup_clear_value = 0xffffffff;
                    writer.writeMemory(intrerrupt_addr_, interrup_clear_value);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", intrerrupt_addr_, interrup_clear_value);
                    printf("mpause 10\n");
            
            
                }
            
            
                // Send Length
                uintptr_t length_addr = 0x43c00018; 
                uint32_t length_value = 0x4;
                writer.writeMemory(length_addr, length_value);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", length_addr, length_value);
                printf("mpause 10\n");
            
                // FIFO Execute : 0x1
                uintptr_t addr_1c = 0x43c0001c;
                uint32_t value_1c = 0x1;
                writer.writeMemory(addr_1c, value_1c);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", addr_1c, value_1c);
                printf("mpause 10\n");
            
                // FIFO 1~8 SEND : 0x1
                uintptr_t send_addr = 0x43c00014;
                uint32_t send_value = 0xff;
                writer.writeMemory(send_addr, send_value);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", send_addr, send_value);
                printf("mpause 10\n");
            
            
                // SEND FIFO CHECK !!!
                while (true)
                {
                    uintptr_t fifo_send_check_address = 0x43c00014;
                    uint32_t fifo_send_check_value;
                    if (!writer.readMemory(fifo_send_check_address, fifo_send_check_value)) {
                        printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(fifo_send_check_address));
                        printf("mpause 10\n");
                        break;
                    }
                
                    if (fifo_send_check_value == 0x0) 
                    {
                        printf(";ok. fifo send completed\n");
                        break;
                    }
                
                    printf("mpause 100\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            
                // 남은 FIFO DATA SIZE 확인
                uintptr_t remaining_fifo_address = 0x43c50000 + 0xc;
                uint32_t remaining_fifo_data_size;
                if (!writer.readMemory(remaining_fifo_address, remaining_fifo_data_size)) {
                    printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(remaining_fifo_address + 0xC));
                    printf("mpause 10\n");
                }
                else
                {
                    printf("@@@ remaining size => 0x%08x @@@\n", remaining_fifo_data_size);
                }
                
                remaining_fifo_address = 0x43c40000 + 0xc;
                if (!writer.readMemory(remaining_fifo_address, remaining_fifo_data_size)) {
                    printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(remaining_fifo_address + 0xC));
                    printf("mpause 10\n");
                }
                else
                {
                    printf("@@@ remaining size => 0x%08x @@@\n", remaining_fifo_data_size);
                }
            
                printf("rx 패널 초기화 끝 !!!!\n");
            #endif

            }
            else
            {
                cout << "Invalid input. Please enter 'tx' or 'rx'." << endl;
                continue;
            }
            
            // 2단계: az 값 입력 받기
            cout << "Enter az value > ";
            string az_input;
            getline(std::cin, az_input);
            
            try {
                az_value = stof(az_input);
            }
            catch(const std::exception& e) {
                cout << "Invalid az value. Please enter a number." << endl;
                continue;
            }
            
            // 3단계: el 값 입력 받기
            cout << "Enter el value > ";
            string el_input;
            getline(std::cin, el_input);

            try 
            {
                el_value = stof(el_input);
                printf("\n++++++++++++++++++++++++\n");
                printf("[sch] start\n");
                printf("++++++++++++++++++++++++\n");
                uintptr_t base_addr = 0x43c40000;
                uintptr_t init_addr = 0x43c4002c;
                uint32_t init_value = 0x2;
                writer.writeMemory(init_addr, init_value);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", init_addr, init_value);
                printf("mpause 10\n");
 
                const int ROWS = 32;
                const int COLS = 32;
            

                struct Entry {
                    int spi_id;
                    int chip_id;
                    int channel_id;
                    double x_offset;
                    double y_offset;
                    int poles;
                    double calculated_phase; // 계산된 phase 값 추가
                    double final_phase;      // offset 적용 후 최종 phase 값 추가
                };
    
                std::vector<Entry> entries;
                std::set<std::tuple<int, int, int>> unique_keys;  // for validation
            
                std::vector<int> channel_pattern_even, channel_pattern_odd;

                if(is_tx == 1)
                {
                    channel_pattern_even = {0x27, 0x3F, 0x47, 0x5F}; // 짝수 col
                    channel_pattern_odd  = {0x5F, 0x47, 0x3F, 0x27}; // 홀수 col
                }
                else
                {
                    channel_pattern_even = {0x22, 0x3A, 0x42, 0x5A}; // 짝수 col
                    channel_pattern_odd  = {0x5A, 0x42, 0x3A, 0x22}; // 홀수 col
                }

                for (int row = 0; row < ROWS; ++row) {
                    for (int col = 0; col < COLS; ++col) 
                    {
                        // spi_id 패턴
                        int spi_id = 7 - (col / 4);
            
                        // chip_id 패턴
                        int chip_id = (col % 4 < 2) ? (16 + row / 2) : (row / 2);

                        // channel_id 패턴
                        int channel_id = (col % 2 == 0)
                            ? channel_pattern_even[row % 4]
                            : channel_pattern_odd[row % 4];

                        // poles 패턴
                        int poles;
                        if (row % 2 == 0) {  // 짝수 row
                            poles = (col % 2 == 0) ? 120 : 30;
                        } else {  // 홀수 row
                            poles = (col % 2 == 0) ? 210 : 300;
                        }

                        // x_offset, y_offset 계산
                        double x_offset_ = col * dx;
                        double y_offset_ = row * dy;

                        unique_keys.insert(key);
                        entries.push_back({spi_id, chip_id, channel_id, x_offset_, y_offset_, poles});
                    }
                }

                // // 정렬: (spi_id, chip_id, channel_id)
                // std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
                //     return std::tie(a.spi_id, a.chip_id, a.channel_id) <
                //             std::tie(b.spi_id, b.chip_id, b.channel_id);
                // });

                // 각 Entry에 대해 Phase 계산 및 Offset 적용
                cnt77 = 0;
                for (auto& e : entries) { // entries를 참조로 순회하여 값 변경
                    // 1. 위상(phase) 계산
                    // std::cout << "=====" << endl; 
                    // std::cout << "x = " << e.x_offset
                    //           << ", y = " << e.y_offset << endl;

                    int row_ = e.y_offset / dy;
                    int col_ = e.x_offset / dx;

                    // std::cout << "!!!!! cnt=" << cnt77
                    //           << " row=" << row_
                    //           << " col=" << col_
                    //           << " x=" << e.x_offset
                    //           << " y=" << e.y_offset;

                    cnt77++;

                    std::cout.flush(); // 버퍼 강제 플러시

                    e.calculated_phase = phase(e.x_offset, e.y_offset);

                    std::cout.flush();

                    // std::cout << "phase = " << e.calculated_phase ;
                              
     
                    // 2. 최종 Phase 계산: calculated_phase + offset_value
                    e.final_phase = e.calculated_phase + e.poles;

                    // std::cout << "e.poles = " << e.poles << endl;
                    // std::cout << "e.final_phase1 = " << e.final_phase << endl;
   
                    // 3. 360을 넘으면 360을 뺍니다. (0-360 범위 유지)
                    // std::fmod를 사용하여 부동 소수점 나머지 연산을 수행합니다.
                    e.final_phase = std::fmod(e.final_phase, 360.0);
                    if (e.final_phase < 0) { // 음수 값이 될 수도 있으므로 0 미만인 경우 360을 더해줍니다.
                        e.final_phase += 360.0;
                    }

                    // std::cout << " phase = " << e.final_phase << endl;
                    // std::cout << "=====" << endl;
    

                }
    
                // 결과 출력
                // std::cout << "--- Calculated Phases with Offset ---\n";
                std::cout << std::fixed << std::setprecision(2); // 소수점 두 자리까지 출력
    
                cnt77 = 0;

                int prev_bus_id = 0;
    
                // 정렬: (spi_id, chip_id, channel_id)
                std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
                    return std::tie(a.spi_id, a.chip_id, a.channel_id) <
                            std::tie(b.spi_id, b.chip_id, b.channel_id);
                });

                for (const auto& e : entries) {
                    // x,y 오프셋으로부터 원래의 row, col을 다시 계산
                    int original_row = e.y_offset / dy;
                    int original_col = e.x_offset / dx;
    
                    // std::cout << ";Row: " << std::setw(2) << original_row
                    //         << ", Col: " << std::setw(2) << original_col
                    //         << " | SPI: " << e.spi_id
                    //         << ", CHIP: " << std::setw(2) << e.chip_id
                    //         << ", CHAN: 0x" << std::hex << std::uppercase << e.channel_id << std::dec
                    //         << ", X: " << std::setw(3) << e.x_offset
                    //         << ", Y: " << std::setw(3) << e.y_offset
                    //         << ", Poles: " << std::setw(3) << e.poles
                    //         << ", Calc Phase: " << std::setw(6) << e.calculated_phase
                    //         << ", Final Phase: " << std::setw(6) << e.final_phase
                    //         << std::endl;

                    int int_phase = int(fmod(e.final_phase + 360.0, 360.0) / 5.625);
                    uint16_t value = 0;

                    if(is_tx == 1)
                    {
                        value = \
                        (( 0 & 0x01 ) << 0 ) \
                        | ((127 & 0x7f ) << 1) \
                        | ((3 & 0x03) << 8 ) \
                        | ((int_phase & 0x3f)<<10);
                    }
                    else
                    {
                        value = \
                        (( 0 & 0x01 ) << 0 ) \
                        | ((1 & 0x1) << 3 ) \
                        | ((63 & 0x3f ) << 4) \
                        | ((int_phase & 0x3f)<<10);
                    }

                    printf(";cnt=%d spi_id=0x%02X chip_id=0x%02X chan_id=0x%02X DATA=0x%04X\n", cnt77,
                        e.spi_id & 0xFF, e.chip_id & 0xFF, e.channel_id, value & 0xFFFF);

                    base_address = 0x43c40000 + (e.spi_id * 0x10000);
                    // printf(";bus_id(%d), base_address(0x%08X)\n", e.spi_id, base_address);

                    byte_queues[e.spi_id].push_back(0x28);
                    byte_queues[e.spi_id].push_back(e.chip_id);
                    byte_queues[e.spi_id].push_back(e.channel_id);
                    byte_queues[e.spi_id].push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
                    byte_queues[e.spi_id].push_back(static_cast<uint8_t>(value & 0xFF));
    
                    // 현재 bus의 큐 처리
                    auto& q = byte_queues[e.spi_id];
                    while (q.size() >= 4) 
                    {
                        uint32_t data = 0;
                        for (int i = 0; i < 4; ++i) {
                            data <<= 8;
                            data |= q.front();
                            q.pop_front();
                        }
                        
                        writer.writeMemory(base_address + 0x0010, data);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        printf(";----count : %d ----\n", count__++);
                        printf("sendln \"devmem 0x%08x 32 0x%02x%02x%02x%02x\"\n",
                            base_address + 0x0010,
                            (data >> 24) & 0xFF,
                            (data >> 16) & 0xFF,
                            (data >> 8) & 0xFF,
                            data & 0xFF);
                        printf("mpause 1\n");
                    }
    
                    if(prev_bus_id != e.spi_id)
                    {
                        change_bus_process(e.spi_id);
                    }
    
                    prev_bus_id = e.spi_id;
    
                }
        
                change_bus_process(prev_bus_id);
    
                std::cout << "\nTotal unique entries: " << entries.size() << std::endl;
    
                printf("++++++++++++++++++++++++\n");
                printf("=======done=====\n");
                printf("++++++++++++++++++++++++\n");
        
                // Send Length
                uintptr_t length_addr = 0x43c00018; 
                uint32_t length_value = 0x5;
                writer.writeMemory(length_addr, length_value);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", length_addr, length_value);
                printf("mpause 10\n");
        
                // FIFO Execute : 0x1
                uintptr_t addr_1c = 0x43c0001c;
                uint32_t value_1c = 0x1;
                writer.writeMemory(addr_1c, value_1c);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", addr_1c, value_1c);
                printf("mpause 10\n");
        
                // FIFO 1~8 SEND : 0Xff
                uintptr_t send_addr = 0x43c00014;
                uint32_t send_value = 0xff;
                writer.writeMemory(send_addr, send_value);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", send_addr, send_value);
                printf("mpause 10\n");
        
        
                // 남은 FIFO DATA SIZE 확인
                while (true)
                {
                    uintptr_t fifo_send_check_address = 0x43c00014;
                    uint32_t fifo_send_check_value;
                    if (!writer.readMemory(fifo_send_check_address, fifo_send_check_value)) {
                        printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(fifo_send_check_address));
                        printf("mpause 10\n");
                        break;
                    }
                
                    if (fifo_send_check_value == 0x0) 
                    {
                        printf(";ok fifo send all completed !\n");
                        break;
                    }
                
                    printf("mpause 100\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
        
                uint32_t fifo_1_remaining_size;
                uint32_t fifo_2_remaining_size;
                uint32_t fifo_3_remaining_size;
                uint32_t fifo_4_remaining_size;
                uint32_t fifo_5_remaining_size;
                uint32_t fifo_6_remaining_size;
                uint32_t fifo_7_remaining_size;
                uint32_t fifo_8_remaining_size;
                writer.readMemory(0x43c4000c, fifo_1_remaining_size);
                printf(";now fifo 1 remaining size -> %d\n", fifo_1_remaining_size);
                printf("mpause 10\n");
                writer.readMemory(0x43c5000c, fifo_2_remaining_size);
                printf(";now fifo 2 remaining size -> %d\n", fifo_2_remaining_size);
                printf("mpause 10\n");
                writer.readMemory(0x43c6000c, fifo_3_remaining_size);
                printf(";now fifo 3 remaining size -> %d\n", fifo_3_remaining_size);
                printf("mpause 10\n");
                writer.readMemory(0x43c7000c, fifo_4_remaining_size);
                printf(";now fifo 4 remaining size -> %d\n", fifo_4_remaining_size);
                printf("mpause 10\n");
                writer.readMemory(0x43c8000c, fifo_5_remaining_size);
                printf(";now fifo 5 remaining size -> %d\n", fifo_5_remaining_size);
                printf("mpause 10\n");
                writer.readMemory(0x43c9000c, fifo_6_remaining_size);
                printf(";now fifo 6 remaining size -> %d\n", fifo_6_remaining_size);
                printf("mpause 10\n");
                writer.readMemory(0x43ca000c, fifo_7_remaining_size);
                printf(";now fifo 7 remaining size -> %d\n", fifo_7_remaining_size);
                printf("mpause 10\n");
                writer.readMemory(0x43cb000c, fifo_8_remaining_size);
                printf(";now fifo 8 remaining size -> %d\n", fifo_8_remaining_size);
                printf("mpause 10\n");
    
                cout << "Processing completed." << endl << endl;
            }
            catch(const std::exception& e) {
                cout << "Invalid el value. Please enter a number." << endl;
                continue;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
};

ConsoleRunner::ConsoleRunner( TransportMap& transport_map, ArrayInfoMap& arraym ) : 
    Runner(transport_map, arraym), 
    impl_(new Impl(*this)) 
{
        // Address total 0x43c00000 => 0x43c40000(bus0) ~ 0x43cb0000(bus7)
        writer.initialize(BASE_ADDR, 0xC0000);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));    

}

ConsoleRunner::~ConsoleRunner() 
{
    delete impl_;
}

void ConsoleRunner::Run()
{
    impl_->Run();
}

void ConsoleRunner::Run(SpiwriteProtocol::MemoryWriter& wr)
{
    impl_->Run(wr);
}

void ConsoleRunner::SetMaxTransferSizeInBytes( int transfer_size )
{
    impl_->transfer_size_in_bytes = transfer_size;
}



}

