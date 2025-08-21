#include <stdexcept>
#include <charconv>
#include "string_util.hpp"
#include "SpiwriteCommand.h"


#include <iostream>
#include <fcntl.h>      // open
#include <unistd.h>     // close
#include <sys/mman.h>   // mmap, munmap
#include <cstring>      // strerror
#include <sstream>      // stringstream

#include <thread>
#include <chrono>

#include <map>                // std::map
#include <deque>              // std::deque
#include <mutex>              // std::mutex, std::lock_guard
#include <condition_variable> // std::condition_variable
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <vector>
#include <string>


#include <zlib.h>
#include <lzma.h>


constexpr uintptr_t BASE_ADDR = 0x43C00000;
constexpr uintptr_t FIFO_ADDR = 0x43C40000;


int count = 0;
int is_init_done = 0;


bool writeMemoryDirect(uintptr_t target_address, uint32_t value) {
    size_t page_size = sysconf(_SC_PAGESIZE);
    uintptr_t page_base = target_address & ~(page_size - 1);
    uintptr_t page_offset = target_address - page_base;
    
    if (page_offset + sizeof(uint32_t) > page_size) {
        std::cerr << "Memory access crosses page boundary" << std::endl;
        return false;
    }
    
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        std::cerr << "Failed to open /dev/mem: " << strerror(errno) << std::endl;
        return false;
    }
    
    void* base = mmap(0, page_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_base);
    if (base == MAP_FAILED) {
        std::cerr << "Failed to mmap: " << strerror(errno) << std::endl;
        close(fd);
        return false;
    }
    
    volatile uint32_t* addr = reinterpret_cast<volatile uint32_t*>(
        static_cast<uint8_t*>(base) + page_offset);
        
    // std::cout << "Writing 0x" << std::hex << value 
    //           << " to address 0x" << target_address << std::endl;
              
    __sync_synchronize();
    *addr = value;
    __sync_synchronize();
    
    munmap(base, page_size);
    close(fd);
    return true;
}

namespace SpiBeam {
namespace SpiwriteProtocol {

// 16진수 문자열을 주소로 변환하는 함수
uintptr_t hexStringToAddress(const std::string_view& hex_str) {
    uintptr_t address;
    std::stringstream ss;

    // "0x"로 시작하면 제거 후 변환 시도
    if (hex_str.substr(0, 2) == "0x") {
        ss << std::hex << hex_str.substr(2);
    } else {
        ss << std::hex << hex_str;
    }

    ss >> address;
    return address;
}

// 메모리 값을 직접 uint32_t로 반환하는 함수 (더 효율적)
uint32_t readMemoryValue(uintptr_t target_address) {
    size_t page_size = sysconf(_SC_PAGESIZE);
    uintptr_t page_base = target_address & ~(page_size - 1);
    uintptr_t page_offset = target_address - page_base;
    
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        std::cerr << "Failed to open /dev/mem: " << strerror(errno) << std::endl;
        return 0xFFFFFFFF; // 에러 값
    }
    
    void* mapped_base = mmap(0, page_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, page_base);
    if (mapped_base == MAP_FAILED) {
        std::cerr << "Failed to mmap: " << strerror(errno) << std::endl;
        close(mem_fd);
        return 0xFFFFFFFF; // 에러 값
    }
    
    if (page_offset + sizeof(uint32_t) > page_size) {
        std::cerr << "Memory access would cross page boundary" << std::endl;
        munmap(mapped_base, page_size);
        close(mem_fd);
        return 0xFFFFFFFF; // 에러 값
    }
    
    uint32_t* mapped_addr = (uint32_t*)((uint8_t*)mapped_base + page_offset);
    uint32_t value = *mapped_addr;
    
    munmap(mapped_base, page_size);
    close(mem_fd);
    
    return value;
}


void busyWaitUsingValue(uintptr_t BUSY_CHECK) {
    std::cout << "Waiting for memory address 0x" << std::hex << BUSY_CHECK << " to become 0..." << std::endl;
    
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        
        uint32_t memValue = readMemoryValue(BUSY_CHECK);
        if (memValue == 0xFFFFFFFF) {
            std::cerr << "Failed to read memory, exiting..." << std::endl;
            break;
        }
        
        // 0이 아닌 값이면 계속 대기 (busy)
        if (memValue != 0) {
            std::cout << "Memory busy (value: 0x" << std::hex << memValue << "), waiting..." << std::endl;
            continue;
        } else {
            std::cout << "Memory is free (value: 0x" << std::hex << memValue << "), proceeding..." << std::endl;
            break;
        }
    }
}


bool MemoryWriter::initialize(uintptr_t base_addr, size_t size) {
    size_t page_size = sysconf(_SC_PAGESIZE);
    uintptr_t page_base = base_addr & ~(page_size - 1);
    mapped_size = ((size + page_size - 1) / page_size) * page_size;
    
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        std::cerr << "Failed to open /dev/mem: " << strerror(errno) << std::endl;
        return false;
    }
    
    mapped_base = mmap(0, mapped_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, page_base);
    if (mapped_base == MAP_FAILED) {
        std::cerr << "Failed to mmap: " << strerror(errno) << std::endl;
        close(mem_fd);
        mem_fd = -1;
        return false;
    }
    
    mapped_address = page_base;
    return true;
}

bool MemoryWriter::readMemory(uintptr_t target_address, uint32_t& out_value) 
{
    std::lock_guard<std::mutex> lock(write_mutex); // write_mutex 재사용

    // 정렬 확인
    if (target_address % sizeof(uint32_t) != 0) {
        std::cerr << "Address not aligned: 0x" << std::hex << target_address << std::endl;
        return false;
    }

    // 범위 확인
    if (target_address < mapped_address || 
        target_address + sizeof(uint32_t) > mapped_address + mapped_size) {
        std::cerr << "Address out of mapped range: 0x" << std::hex << target_address << std::endl;
        return false;
    }

    volatile uint32_t* addr = reinterpret_cast<volatile uint32_t*>(
        static_cast<uint8_t*>(mapped_base) + (target_address - mapped_address));

    __sync_synchronize();
    out_value = *addr;
    __sync_synchronize();

    return true;
}

bool MemoryWriter::writeMemory(uintptr_t target_address, uint32_t value) {
    if (mem_fd == -1 || mapped_base == nullptr) {
        // 초기화되지 않은 경우 간단한 방식으로 처리
        return writeMemoryDirect(target_address, value);
    }
    
    std::lock_guard<std::mutex> lock(write_mutex);
    
    // 정렬 확인
    if (target_address % sizeof(uint32_t) != 0) {
        std::cerr << "Address not aligned: 0x" << std::hex << target_address << std::endl;
        return false;
    }
    
    // 범위 확인
    if (target_address < mapped_address || 
        target_address + sizeof(uint32_t) > mapped_address + mapped_size) {
        std::cerr << "Address out of mapped range: 0x" << std::hex << target_address << std::endl;
        return false;
    }
    
    volatile uint32_t* addr = reinterpret_cast<volatile uint32_t*>(
        static_cast<uint8_t*>(mapped_base) + (target_address - mapped_address));
        
    // std::cout << "Writing 0x" << std::hex << value 
    //           << " to address 0x" << target_address << std::endl;
              
    __sync_synchronize();
    *addr = value;
    __sync_synchronize();
    
    return true;
}

std::map<int, std::deque<uint8_t>> byte_queues;
std::map<int, std::mutex> queue_mutexes;



enum class CommandType : uint8_t {
    BULK_WRITE = 0x01,
    // 다른 명령어 타입들 (필요시 추가)
    // SINGLE_WRITE = 0x02,
    // BULK_READ = 0x03,
    // etc...
};

struct BinaryWriteCommand {
    uint8_t bus;
    uint8_t addr;
    uint8_t reg;
    uint16_t value;
};


int count__ = 1;
Result SpiwriteCommand::parse_binary_commands(const std::vector<uint8_t>& binary_data) {     
    size_t available_data = binary_data.size() - 3;
    size_t max_possible_cmds = available_data / 2;    
    size_t offset = 3;
    int parsed_count = 1;
    uint8_t bus_id = 0;
    uint8_t ucIcAddr = 0;
    uint8_t ucRegisterAddr;
    uint16_t value;
    uintptr_t base_address;
    int previous_bus_id = 0;
    uintptr_t prev_base_address;
    const std::vector<uint8_t> register_addrs = {0x27, 0x3F, 0x47, 0x5F};
    size_t reg_idx = 0;
    int command_count = 1;
    
    while (offset + 1 <= binary_data.size()) 
    {
        ucRegisterAddr = register_addrs[reg_idx];
        value = (binary_data[offset] << 8) | binary_data[offset + 1];
        
        base_address = 0x43c40000 + (bus_id * 0x10000);
        printf(";bus_id(%d), base_address(0x%08X)\n", bus_id, base_address);

        // 현재 bus_id로 데이터를 큐에 추가
        byte_queues[bus_id].push_back(0x28);
        byte_queues[bus_id].push_back(ucIcAddr);
        byte_queues[bus_id].push_back(ucRegisterAddr);
        byte_queues[bus_id].push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        byte_queues[bus_id].push_back(static_cast<uint8_t>(value & 0xFF));

        // 현재 bus의 큐 처리
        auto& q = byte_queues[bus_id];
        while (q.size() >= 4) 
        {
            uint32_t data = 0;
            for (int i = 0; i < 4; ++i) {
                data <<= 8;
                data |= q.front();
                q.pop_front();
            }
            
            wr.writeMemory(base_address + 0x0010, data);
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
            printf(";----count : %d ----\n", count__++);
            printf("sendln \"devmem 0x%08x 32 0x%02x%02x%02x%02x\"\n",
                   base_address + 0x0010,
                   (data >> 24) & 0xFF,
                   (data >> 16) & 0xFF,
                   (data >> 8) & 0xFF,
                   data & 0xFF);
            printf("mpause 10\n");
            
            // printf("mpause 1\n");
        }

        // 다음 레지스터/IC/버스 계산
        reg_idx++;
        if (reg_idx >= 4) 
        {
            reg_idx = 0;
            ucIcAddr++;
            if (ucIcAddr > 0x1F) 
            {
                ucIcAddr = 0;
                
                // 현재 bus 종료 처리
                wr.writeMemory(base_address + 0x0014, 0x280);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x280\"\n", base_address + 0x0014);
                printf("mpause 10\n");

                // 인터럽트 상태 확인
                uint32_t interrupt_value;
                wr.readMemory(base_address, interrupt_value);
                printf(";base_address => 0X%08x interrupt_value => 0x%08x\n", base_address, interrupt_value);
                printf("mpause 10\n");
                // while (true)
                // {
                //     uint32_t interrupt_value;
                //     if (!wr.readMemory(base_address, interrupt_value)) {
                //         printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(base_address));
                //         break;
                //     }
                
                //     if (interrupt_value == 0) 
                //     {
                //         printf(";interrupt check ok !\n");
                //         break;
                //     }
                
                //     printf("mpause 100\n");
                //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // }

                // 인터럽트 초기화
                wr.writeMemory(base_address, 0xffffffff);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0xffffffff\"\n", base_address);
                printf("mpause 10\n");

                // 남은 FIFO DATA SIZE 확인
                uint32_t remaining_fifo_data_size;
                wr.readMemory(base_address + 0xC, remaining_fifo_data_size);
                printf(";remaining_fifo_data_size => 0x%08x\n", remaining_fifo_data_size);
                printf("mpause 10\n");

                // while (true)
                // {
                //     uint32_t remaining_fifo_data_size;
                //     if (!wr.readMemory(base_address + 0xC, remaining_fifo_data_size)) {
                //         printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(base_address));
                //         break;
                //     }
                
                //     if (remaining_fifo_data_size == 0x15E) 
                //     {
                //         printf(";remaining_fifo_data_size is 0x15E ok!\n");
                //         break;
                //     }
                
                //     printf("mpause 100\n");
                //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // }

                // 다음 bus로 이동
                bus_id++;
                if (bus_id > 0x07) {
                    break;
                }
                
                // 새로운 bus 시작 처리
                base_address = 0x43c40000 + (bus_id * 0x10000);
                uint32_t init_value_ = 0x2;
                wr.writeMemory(base_address + 0x2C, init_value_);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                printf("sendln \"devmem 0x%08x 32 0x%01X\"\n", base_address + 0x2C, init_value_);
                printf("mpause 10\n");
                
                // count 리셋
                count__ = 1;
            }
        }

        offset += 2;
        parsed_count++;
    }

    return Result{"001"};
}

// 기본 생성자 구현
SpiwriteCommand::SpiwriteCommand(Controller::Transport& transport, 
    Controller::CodeGenerator* cgen, 
    Parser::LineParser* parser)
: transport_(transport), code_generator_(cgen), parser_(parser)
{
    // Address total 0x43c00000 => 0x43c40000(bus0) ~ 0x43cb0000(bus7)
    wr.initialize(BASE_ADDR, 0xC0000);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

 #if 0
    printf("rx 패널 초기화 시작 !!!!\n");

    // VAIC RESET OFF
    uintptr_t vaic_reset_addr = 0x43c28004;
    uint32_t reset_off = 0xff;
    wr.writeMemory(vaic_reset_addr, reset_off);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", vaic_reset_addr, reset_off);
    printf("mpause 10\n");

    // VAIC RESET ON
    uint32_t reset_on = 0x0;
    wr.writeMemory(vaic_reset_addr, reset_on);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", vaic_reset_addr, reset_on);
    printf("mpause 10\n");

    // FIFO init
    for (int i = 0; i < 8; i++) 
    {
        uintptr_t fifo_addr = FIFO_ADDR + (i * 0x10000);

        // 인터럽트 상태 확인
        uint32_t interrupt_value;
        wr.readMemory(fifo_addr, interrupt_value);
        printf(";interrupt check addr -> 0x%08x, value -> 0x%08x\n", fifo_addr, interrupt_value);
        printf("mpause 10\n");

        // while (true)
        // {
        //     uint32_t interrupt_value;
        //     if (!wr.readMemory(fifo_addr, interrupt_value)) {
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
        wr.writeMemory(fifo_addr, interrup_clear_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_addr, interrup_clear_value);
        printf("mpause 10\n");
    }


    for(int j = 0; j < 8; j++)
    {
        // start address
        uintptr_t start_address_ = FIFO_ADDR + (i * 0x10000) + 0x2C;
        uint32_t start_address_value__ = 0x2;
        wr.writeMemory(start_address_, start_address_value__);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", start_address_, start_address_value__);
        printf("mpause 10\n");


        uintptr_t fifo_1_address_ = FIFO_ADDR + (i * 0x10000) + 0x10;
        uint32_t broadcast_value1 = 0x60000000;
        wr.writeMemory(fifo_1_address_, broadcast_value1);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value1);
        printf("mpause 10\n");
        uint32_t broadcast_value2 = 0x6001068A;
        wr.writeMemory(fifo_1_address_, broadcast_value2);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value2);
        printf("mpause 10\n");

        ////////////////////////////////////
        // 0x25, 3D, 45, 5D
        ////////////////////////////////////
        uint32_t broadcast_value3 = 0x60206CDB;
        wr.writeMemory(fifo_1_address_, broadcast_value3);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value3);
        printf("mpause 10\n");
        uint32_t broadcast_value4 = 0x60386CDB;
        wr.writeMemory(fifo_1_address_, broadcast_value4);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value4);
        printf("mpause 10\n");
        uint32_t broadcast_value5 = 0x60406CDB;
        wr.writeMemory(fifo_1_address_, broadcast_value5);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value5);
        printf("mpause 10\n");
        uint32_t broadcast_value6 = 0x60586CDB;
        wr.writeMemory(fifo_1_address_, broadcast_value6);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value6);
        printf("mpause 10\n");

        ////////////////////////////////////
        // 0x26, 3E, 46, 5E
        ////////////////////////////////////
        uint32_t broadcast_value7 = 0x60212FFF;
        wr.writeMemory(fifo_1_address_, broadcast_value7);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value7);
        printf("mpause 10\n");
        uint32_t broadcast_value8 = 0x60392FFF;
        wr.writeMemory(fifo_1_address_, broadcast_value8);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value8);
        printf("mpause 10\n");
        uint32_t broadcast_value9 = 0x60412FFF;
        wr.writeMemory(fifo_1_address_, broadcast_value9);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value9);
        printf("mpause 10\n");
        uint32_t broadcast_value10 = 0x60592FFF;
        wr.writeMemory(fifo_1_address_, broadcast_value10);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value10);
        printf("mpause 10\n");

        ////////////////////////////////////
        // 0x27, 0x3F, 0x47,  0x5F
        ////////////////////////////////////
        uint32_t broadcast_value11 = 0x602203F8;
        wr.writeMemory(fifo_1_address_, broadcast_value11);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value11);
        printf("mpause 10\n");
        uint32_t broadcast_value12 = 0x603A03F8;
        wr.writeMemory(fifo_1_address_, broadcast_value12);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value12);
        printf("mpause 10\n");
        uint32_t broadcast_value13 = 0x604203F8;
        wr.writeMemory(fifo_1_address_, broadcast_value13);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value13);
        printf("mpause 10\n");
        uint32_t broadcast_value14 = 0x605A03F8;
        wr.writeMemory(fifo_1_address_, broadcast_value14);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value14);
        printf("mpause 10\n");

        // length : 56 Byte (0x38)
        uintptr_t fifo_send_length = FIFO_ADDR + (i * 0x10000) + 0x14;
        uint32_t fifo_send_length____ = 0x38;
        wr.writeMemory(fifo_send_length, fifo_send_length____);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_send_length, fifo_send_length____);
        printf("mpause 10\n");

        // 인터럽트 상태 확인
        uintptr_t intrerrupt_addr___ = FIFO_ADDR + (i * 0x10000);
        uint32_t interrupt_value;
        wr.readMemory(intrerrupt_addr___, interrupt_value);
        printf(";interrupt check addr -> 0x%08x, value -> 0x%08x\n", intrerrupt_addr___, interrupt_value);
        printf("mpause 10\n");

        // while (true)
        // {
        //     uintptr_t intrerrupt_addr = 0x43c40000;
        //     uint32_t interrupt_value;
        //     if (!wr.readMemory(intrerrupt_addr, interrupt_value)) {
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
        wr.writeMemory(intrerrupt_addr_, interrup_clear_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", intrerrupt_addr_, interrup_clear_value);
        printf("mpause 10\n");


    }


    // Send Length
    uintptr_t length_addr = 0x43c00018; 
    uint32_t length_value = 0x4;
    wr.writeMemory(length_addr, length_value);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", length_addr, length_value);
    printf("mpause 10\n");

    // FIFO Execute : 0x1
    uintptr_t addr_1c = 0x43c0001c;
    uint32_t value_1c = 0x1;
    wr.writeMemory(addr_1c, value_1c);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", addr_1c, value_1c);
    printf("mpause 10\n");

    // FIFO 1~8 SEND : 0x1
    uintptr_t send_addr = 0x43c00014;
    uint32_t send_value = 0xff;
    wr.writeMemory(send_addr, send_value);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", send_addr, send_value);
    printf("mpause 10\n");


    // SEND FIFO CHECK !!!
    while (true)
    {
        uintptr_t fifo_send_check_address = 0x43c00014;
        uint32_t fifo_send_check_value;
        if (!wr.readMemory(fifo_send_check_address, fifo_send_check_value)) {
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
    if (!wr.readMemory(remaining_fifo_address, remaining_fifo_data_size)) {
        printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(remaining_fifo_address + 0xC));
        printf("mpause 10\n");
    }
    else
    {
        printf("@@@ remaining size => 0x%08x @@@\n", remaining_fifo_data_size);
    }
    
    remaining_fifo_address = 0x43c40000 + 0xc;
    if (!wr.readMemory(remaining_fifo_address, remaining_fifo_data_size)) {
        printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(remaining_fifo_address + 0xC));
        printf("mpause 10\n");
    }
    else
    {
        printf("@@@ remaining size => 0x%08x @@@\n", remaining_fifo_data_size);
    }

    printf("rx 패널 초기화 끝 !!!!\n");
#endif

#if 0
    printf("\n@@@ tx 패널 초기화 시작 @@@\n");

    // VAIC RESET OFF
    uintptr_t vaic_reset_addr = 0x43c28004;
    uint32_t reset_off = 0xff;
    wr.writeMemory(vaic_reset_addr, reset_off);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", vaic_reset_addr, reset_off);
    printf("mpause 10\n");

    // VAIC RESET ON
    uint32_t reset_on = 0x0;
    wr.writeMemory(vaic_reset_addr, reset_on);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", vaic_reset_addr, reset_on);
    printf("mpause 10\n");

    // FIFO init
    for (int i = 0; i < 8; i++) 
    {
        uintptr_t fifo_addr = FIFO_ADDR + (i * 0x10000);

        // 인터럽트 상태 확인
        uint32_t interrupt_value;
        wr.readMemory(fifo_addr, interrupt_value);
        printf(";interrupt check addr -> 0x%08x, value -> 0x%08x\n", fifo_addr, interrupt_value);
        printf("mpause 10\n");

        // while (true)
        // {
        //     uint32_t interrupt_value;
        //     if (!wr.readMemory(fifo_addr, interrupt_value)) {
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
        wr.writeMemory(fifo_addr, interrup_clear_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_addr, interrup_clear_value);
        printf("mpause 10\n");
    }

    for(int i = 0; i < 8; i++)
    {
        // start address
        uintptr_t start_address_ = FIFO_ADDR + (i * 0x10000) + 0x2C;
        uint32_t start_address_value__ = 0x2;
        wr.writeMemory(start_address_, start_address_value__);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", start_address_, start_address_value__);
        printf("mpause 10\n");


        uintptr_t fifo_1_address_ = FIFO_ADDR + (i * 0x10000) + 0x10;
        uint32_t broadcast_value1 = 0x60000000;
        wr.writeMemory(fifo_1_address_, broadcast_value1);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value1);
        printf("mpause 10\n");
        uint32_t broadcast_value2 = 0x60010688;
        wr.writeMemory(fifo_1_address_, broadcast_value2);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value2);
        printf("mpause 10\n");

        ////////////////////////////////////
        // 0x25, 3D, 45, 5D
        ////////////////////////////////////
        uint32_t broadcast_value3 = 0x6025A91A;
        wr.writeMemory(fifo_1_address_, 0x6025A91A);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value3);
        printf("mpause 10\n");
        uint32_t broadcast_value4 = 0x603DA91A;
        wr.writeMemory(fifo_1_address_, 0x603DA91A);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value4);
        printf("mpause 10\n");
        uint32_t broadcast_value5 = 0x6045A91A;
        wr.writeMemory(fifo_1_address_, 0x6045A91A);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value5);
        printf("mpause 10\n");
        uint32_t broadcast_value6 = 0x605DA91A;
        wr.writeMemory(fifo_1_address_, 0x605DA91A);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value6);
        printf("mpause 10\n");

        ////////////////////////////////////
        // 0x26, 3E, 46, 5E
        ////////////////////////////////////
        uint32_t broadcast_value7 = 0x60260E7F;
        wr.writeMemory(fifo_1_address_, 0x60260E7F);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value7);
        printf("mpause 10\n");
        uint32_t broadcast_value8 = 0x603E0E7F;
        wr.writeMemory(fifo_1_address_, 0x603E0E7F);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value8);
        printf("mpause 10\n");
        uint32_t broadcast_value9 = 0x60460E7F;
        wr.writeMemory(fifo_1_address_, 0x60460E7F);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value9);
        printf("mpause 10\n");
        uint32_t broadcast_value10 = 0x605E0E7F;
        wr.writeMemory(fifo_1_address_, 0x605E0E7F);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value10);
        printf("mpause 10\n");

        ////////////////////////////////////
        // 0x27, 0x3F, 0x47,  0x5F
        ////////////////////////////////////
        uint32_t broadcast_value11 = 0x602703FE;
        wr.writeMemory(fifo_1_address_, 0x602703FE);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value11);
        printf("mpause 10\n");
        uint32_t broadcast_value12 = 0x603F03FE;
        wr.writeMemory(fifo_1_address_, 0x603F03FE);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value12);
        printf("mpause 10\n");
        uint32_t broadcast_value13 = 0x604703FE;
        wr.writeMemory(fifo_1_address_, 0x604703FE);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value13);
        printf("mpause 10\n");
        uint32_t broadcast_value14 = 0x605F03FE;
        wr.writeMemory(fifo_1_address_, 0x605F03FE);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_1_address_, broadcast_value14);
        printf("mpause 10\n");

        // length : 56 Byte (0x38)
        uintptr_t fifo_send_length = FIFO_ADDR + (i * 0x10000) + 0x14;
        uint32_t fifo_send_length____ = 0x38;
        wr.writeMemory(fifo_send_length, fifo_send_length____);
        printf("sendln \"devmem 0x%08x 32 0x%08x\"\n", fifo_send_length, fifo_send_length____);
        printf("mpause 10\n");

        // 인터럽트 상태 확인
        uintptr_t intrerrupt_addr___ = FIFO_ADDR + (i * 0x10000);
        uint32_t interrupt_value;
        wr.readMemory(intrerrupt_addr___, interrupt_value);
        printf(";interrupt check addr -> 0x%08x, value -> 0x%08x\n", intrerrupt_addr___, interrupt_value);
        printf("mpause 10\n");

        // while (true)
        // {
        //     uintptr_t intrerrupt_addr = 0x43c40000;
        //     uint32_t interrupt_value;
        //     if (!wr.readMemory(intrerrupt_addr, interrupt_value)) {
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
        wr.writeMemory(intrerrupt_addr_, interrup_clear_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", intrerrupt_addr_, interrup_clear_value);
        printf("mpause 10\n");


    }


    // Send Length
    uintptr_t length_addr = 0x43c00018; 
    uint32_t length_value = 0x4;
    wr.writeMemory(length_addr, length_value);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", length_addr, length_value);
    printf("mpause 10\n");

    // FIFO Execute : 0x1
    uintptr_t addr_1c = 0x43c0001c;
    uint32_t value_1c = 0x1;
    wr.writeMemory(addr_1c, value_1c);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", addr_1c, value_1c);
    printf("mpause 10\n");

    // FIFO 1~8 SEND : 0x1
    uintptr_t send_addr = 0x43c00014;
    uint32_t send_value = 0xff;
    wr.writeMemory(send_addr, send_value);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", send_addr, send_value);
    printf("mpause 10\n");


    // SEND FIFO CHECK !!!
    while (true)
    {
        uintptr_t fifo_send_check_address = 0x43c00014;
        uint32_t fifo_send_check_value;
        if (!wr.readMemory(fifo_send_check_address, fifo_send_check_value)) {
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
    if (!wr.readMemory(remaining_fifo_address, remaining_fifo_data_size)) {
        printf("Failed to read memory at address 0x%08lx\n", static_cast<unsigned long>(remaining_fifo_address + 0xC));
        printf("mpause 10\n");
    }
    else
    {
        printf("@@@ remaining size => 0x%08x @@@\n", remaining_fifo_data_size);
    }
    
    remaining_fifo_address = 0x43c40000 + 0xc;
    if (!wr.readMemory(remaining_fifo_address, remaining_fifo_data_size)) {
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

//기존 텍스트 명령어 파싱
Result SpiwriteCommand::parse_text_commands(const std::vector<std::string_view>& tokens) {
    std::string cmd( tokens[0] );

    if ( cmd == "start")
    {
        printf("\n++++++++++++++++++++++++\n");
        printf("[sch] start\n");
        printf("++++++++++++++++++++++++\n");

        count = 0;
        uintptr_t base_addr = 0x43c40000;
        uintptr_t init_addr = 0x43c4002c;
        uint32_t init_value = 0x2;
        wr.writeMemory(init_addr, init_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", init_addr, init_value);
        printf("mpause 10\n");

        return Result{ "stat init completed !!!" };
    }

    if ( cmd == "done")
    {
        printf("++++++++++++++++++++++++\n");
        printf("=======axi_fifo_write_done=====\n");
        printf("++++++++++++++++++++++++\n");

        // Send Length
        uintptr_t length_addr = 0x43c00018; 
        uint32_t length_value = 0x5;
        wr.writeMemory(length_addr, length_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", length_addr, length_value);
        printf("mpause 10\n");

        // FIFO Execute : 0x1
        uintptr_t addr_1c = 0x43c0001c;
        uint32_t value_1c = 0x1;
        wr.writeMemory(addr_1c, value_1c);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", addr_1c, value_1c);
        printf("mpause 10\n");

        // FIFO 1~8 SEND : 0Xff
        uintptr_t send_addr = 0x43c00014;
        uint32_t send_value = 0xff;
        wr.writeMemory(send_addr, send_value);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("sendln \"devmem 0x%08x 32 0x%01x\"\n", send_addr, send_value);
        printf("mpause 10\n");


        // 남은 FIFO DATA SIZE 확인
        while (true)
        {
            uintptr_t fifo_send_check_address = 0x43c00014;
            uint32_t fifo_send_check_value;
            if (!wr.readMemory(fifo_send_check_address, fifo_send_check_value)) {
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
        wr.readMemory(0x43c4000c, fifo_1_remaining_size);
        printf(";now fifo 1 remaining size -> %d\n", fifo_1_remaining_size);
        printf("mpause 10\n");
        wr.readMemory(0x43c5000c, fifo_2_remaining_size);
        printf(";now fifo 2 remaining size -> %d\n", fifo_2_remaining_size);
        printf("mpause 10\n");
        wr.readMemory(0x43c6000c, fifo_3_remaining_size);
        printf(";now fifo 3 remaining size -> %d\n", fifo_3_remaining_size);
        printf("mpause 10\n");
        wr.readMemory(0x43c7000c, fifo_4_remaining_size);
        printf(";now fifo 4 remaining size -> %d\n", fifo_4_remaining_size);
        printf("mpause 10\n");
        wr.readMemory(0x43c8000c, fifo_5_remaining_size);
        printf(";now fifo 5 remaining size -> %d\n", fifo_5_remaining_size);
        printf("mpause 10\n");
        wr.readMemory(0x43c9000c, fifo_6_remaining_size);
        printf(";now fifo 6 remaining size -> %d\n", fifo_6_remaining_size);
        printf("mpause 10\n");
        wr.readMemory(0x43ca000c, fifo_7_remaining_size);
        printf(";now fifo 7 remaining size -> %d\n", fifo_7_remaining_size);
        printf("mpause 10\n");
        wr.readMemory(0x43cb000c, fifo_8_remaining_size);
        printf(";now fifo 8 remaining size -> %d\n", fifo_8_remaining_size);
        printf("mpause 10\n");

        return Result{"done complete"};

    }
    
    return Result {"what?"};
}

// zlib 헤더 검증 함수
bool validate_zlib_header(const std::vector<uint8_t>& data) {
    if (data.size() < 2) {
        printf("Data too small for zlib header\n");
        return false;
    }
    
    uint8_t cmf = data[0];  // 0x78
    uint8_t flg = data[1];  // 0x9C
    
    printf("=== zlib Header Analysis ===\n");
    printf("CMF: 0x%02X\n", (int)cmf);
    printf("FLG: 0x%02X\n", (int)flg);
    
    // Compression method (하위 4비트)
    uint8_t cm = cmf & 0x0F;
    printf("Compression Method: %d", (int)cm);
    if (cm == 8) {
        printf(" (deflate) o\n");
    } else {
        printf(" (invalid) x\n");
        return false;
    }
    
    // Compression info (상위 4비트)
    uint8_t cinfo = (cmf >> 4) & 0x0F;
    uint32_t window_size = 1 << (cinfo + 8);
    printf("Window Size: %u bytes (2^%d)\n", window_size, (cinfo + 8));
    
    // FLG 분석
    uint8_t fcheck = flg & 0x1F;
    uint8_t fdict = (flg >> 5) & 0x01;
    uint8_t flevel = (flg >> 6) & 0x03;
    
    printf("FCHECK: %d\n", (int)fcheck);
    printf("FDICT: %d%s\n", (int)fdict, fdict ? " (dictionary present)" : " (no dictionary)");
    printf("FLEVEL: %d", (int)flevel);
    switch (flevel) {
        case 0: printf(" (fastest)"); break;
        case 1: printf(" (fast)"); break;
        case 2: printf(" (default)"); break;
        case 3: printf(" (maximum)"); break;
    }
    printf("\n");
    
    // 체크섬 검증
    uint16_t header_checksum = (cmf << 8) + flg;
    printf("Header checksum: %u %% 31 = %u", header_checksum, (header_checksum % 31));
    if (header_checksum % 31 == 0) {
        printf(" o\n");
    } else {
        printf(" x\n");
        return false;
    }
    
    printf("=== Header validation: PASSED  ===\n");
    return true;
}

// 개선된 압축 해제 함수 (상세한 로그 포함)
std::vector<uint8_t> decompress_zlib_verbose(const std::vector<uint8_t>& compressed_data) {
    printf("\n=== Starting zlib decompression ===\n");
    printf("Input size: %zu bytes\n", compressed_data.size());
    
    // 헤더 검증
    if (!validate_zlib_header(compressed_data)) 
    {
        printf("Invalid zlib header\n");
        throw std::runtime_error("Invalid zlib header");
    }
    
    z_stream strm;
    memset(&strm, 0, sizeof(strm));
    
    // zlib 형식 사용 (15는 최대 윈도우 크기)
    int init_result = inflateInit2(&strm, 15);
    if (init_result != Z_OK) 
    {
        printf("zlib initialization failed\n");
        throw std::runtime_error("zlib initialization failed: " + std::to_string(init_result));
    }
    
    std::vector<uint8_t> decompressed;
    const size_t chunk_size = 32768;
    
    strm.next_in = const_cast<Bytef*>(compressed_data.data());
    strm.avail_in = compressed_data.size();
    
    int ret;
    int chunk_count = 0;
    size_t total_output = 0;
    
    printf("\n=== Decompression process ===\n");
    
    do {
        size_t old_size = decompressed.size();
        decompressed.resize(old_size + chunk_size);
        
        strm.next_out = decompressed.data() + old_size;
        strm.avail_out = chunk_size;
        
        ret = inflate(&strm, Z_NO_FLUSH);
        
        chunk_count++;
        size_t bytes_written = chunk_size - strm.avail_out;
        total_output += bytes_written;
        
        printf("Chunk %d: ret=%d, bytes_written=%zu, avail_in=%u, total_out=%zu\n", 
               chunk_count, ret, bytes_written, strm.avail_in, total_output);
        
        if (ret == Z_STREAM_ERROR || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
            inflateEnd(&strm);
            std::string error_msg = "Decompression failed at chunk " + std::to_string(chunk_count) + ": ";
            switch (ret) {
                case Z_STREAM_ERROR: error_msg += "stream error"; break;
                case Z_DATA_ERROR: error_msg += "data error (corrupt data)"; break;
                case Z_MEM_ERROR: error_msg += "memory error"; break;
                default: error_msg += "unknown error (" + std::to_string(ret) + ")"; break;
            }
            throw std::runtime_error(error_msg);
        }
        
        decompressed.resize(old_size + bytes_written);
        
    } while (ret != Z_STREAM_END && strm.avail_in > 0);
    
    if (ret == Z_STREAM_END) {
        printf("✓ Decompression completed successfully!\n");
        printf("Final output size: %zu bytes\n", decompressed.size());
        printf("Compression ratio: %.2f%%\n", (double)compressed_data.size() / decompressed.size() * 100);
    } else {
        inflateEnd(&strm);
        throw std::runtime_error("Decompression incomplete: " + std::to_string(ret));
    }
    
    inflateEnd(&strm);
    return decompressed;
}
 

Result SpiwriteCommand::Execute(const std::string& raw_command) 
{
    // 바이너리 명령어 체크
    if (raw_command.substr(0, 7) == "BINARY:") 
    {
        // 압축된 바이너리 데이터 체크
        size_t binary_start_pos = 7;
        std::string compression_type = "";

        printf("compressed size = %d\n", raw_command.size());
        // "BINARY:" + 최소 2바이트 헤더
        uint8_t first_byte = static_cast<uint8_t>(raw_command[7]);
        uint8_t second_byte = static_cast<uint8_t>(raw_command[8]);
        
        // zlib 매직 헤더 확인 (일반적으로 0x78로 시작)
        if (first_byte == 0x78 && (second_byte & 0x20) == 0) { // FDICT=0 확인
            compression_type = "zlib";
            binary_start_pos = 7; // "BINARY:" 바로 뒤부터
            printf("Detected zlib compressed data by magic header: %02X %02X\n", first_byte, second_byte);
        }

        // 바이너리 데이터 추출 (안전한 방법)
        const char* binary_start = raw_command.data() + binary_start_pos;
        size_t binary_size = raw_command.size() - binary_start_pos;
        
        // 디버깅 정보 출력
        printf("Raw command size: %zu\n", raw_command.size());
        printf("Binary start position: %zu\n", binary_start_pos);
        printf("Binary size: %zu\n", binary_size);
        
        if (binary_size == 0) {
            fprintf(stderr, "No binary data found!\n");
            return Result{"No binary data found"};
        }
        
        // 바이너리 데이터를 안전하게 복사
        std::vector<uint8_t> binary_data;
        binary_data.reserve(binary_size);
        for (size_t i = 0; i < binary_size; ++i) {
            binary_data.push_back(static_cast<uint8_t>(binary_start[i]));
        }
        
        // 바이너리 데이터 헥스 덤프 (처음 16바이트만)
        printf("Binary data hex dump (first 16 bytes): ");
        for (size_t i = 0; i < std::min(binary_size, (size_t)16); ++i) {
            printf("%02X ", binary_data[i]);
        }
        printf("\n");
        
        // 압축된 데이터 처리
        if (!compression_type.empty()) 
        {
            printf("not empty\n");
            try {
                printf("Decompressing %s data...\n", compression_type.c_str());
                printf("Compressed size: %zu bytes\n", binary_data.size());
                
                // zlib 헤더 확인
                if (compression_type == "zlib") {
                    if (binary_data.size() < 2) {
                        printf("Invalid zlib data: too short\n");
                        return Result{"Invalid zlib data: too short"};
                    }
                    
                    // zlib 매직 헤더 확인 (일반적으로 0x78로 시작)
                    uint8_t cmf = binary_data[0];
                    uint8_t flg = binary_data[1];
                    
                    printf("zlib header: CMF=0x%02X, FLG=0x%02X\n", (int)cmf, (int)flg);
                    
                    // zlib 헤더 검증
                    if ((cmf & 0x0F) != 8) { // deflate method
                        printf("Invalid zlib compression method\n");
                        return Result{"Invalid zlib compression method"};
                    }
                    
                    if (((cmf << 8) + flg) % 31 != 0) {
                        printf("Invalid zlib header checksum\n");
                        return Result{"Invalid zlib header checksum"};
                    }
                }
                
                std::vector<uint8_t> decompressed_data;
                if (compression_type == "zlib") 
                {
                    printf("zlib@@\n");
                    decompressed_data = decompress_zlib_verbose(binary_data);
                }
                
                printf("Decompressed size: %zu bytes\n", decompressed_data.size());
                if (binary_data.size() > 0) {
                    printf("Compression ratio: %.2f%%\n", (double)binary_data.size() / decompressed_data.size() * 100);
                }
                
                // 압축 해제된 데이터 헥스 덤프 (처음 16바이트만)
                printf("Decompressed data hex dump (first 16 bytes): ");
                for (size_t i = 0; i < std::min(decompressed_data.size(), (size_t)16); ++i) {
                    printf("%02X ", decompressed_data[i]);
                }
                printf("\n");
                
                // 압축 해제된 바이너리 명령어 처리
                return parse_binary_commands(decompressed_data);
                
            } catch (const std::exception& e) {
                fprintf(stderr, "Decompression error: %s\n", e.what());
                return Result{"Decompression error: " + std::string(e.what())};
            }
        } else {
            // 기존 비압축 바이너리 데이터 처리
            printf("empty !!!\n");
            return parse_binary_commands(binary_data);
        }
    }

    // 기존 텍스트 방식 처리
    // 토큰화 (공백과 & 구분자로 분리)
    std::vector<std::string_view> tokens;
    
    // 간단한 토큰화 (실제로는 더 정교한 파싱 필요)
    size_t start = 0;
    size_t pos = 0;
    
    while (pos < raw_command.length()) {
        if (raw_command[pos] == ' ' || raw_command[pos] == '&') {
            if (pos > start) {
                tokens.emplace_back(raw_command.data() + start, pos - start);
            }
            // 연속된 구분자 스킵
            while (pos < raw_command.length() && 
                   (raw_command[pos] == ' ' || raw_command[pos] == '&')) {
                pos++;
            }
            start = pos;
        } else {
            pos++;
        }
    }

    // 마지막 토큰 추가
    if (start < raw_command.length()) {
        tokens.emplace_back(raw_command.data() + start, raw_command.length() - start);
    }
    
    return parse_text_commands(tokens);
}



}
}
