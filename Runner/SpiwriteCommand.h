#ifndef __SPIBEAM_SPIWRITE_COMMNAD_H__
#define __SPIBEAM_SPIWRITE_COMMNAD_H__

#include <string.h>
#include <vector>
#include <algorithm>
#include "Transport.h"
#include "CodeGenerator.h"
#include "LineParser.h"
#include <mutex>        // std::mutex를 위해 필요
#include <memory>       // std::unique_ptr를 위해 필요
#include <unistd.h>     // close(), sysconf()를 위해 필요
#include <sys/mman.h>   // mmap(), munmap()을 위해 필요
#include <fcntl.h>      // open()을 위해 필요
#include <cstdint>      // uintptr_t, uint32_t를 위해 필요
#include <iostream>     // std::cerr를 위해 필요
#include <cstring>      // strerror()를 위해 필요
#include <cerrno>       // errno를 위해 필요

namespace SpiBeam {
namespace SpiwriteProtocol {


struct Result
{
    std::string message;
    std::vector<uint32_t> responses;
};

class MemoryWriter {
    private:
        int mem_fd = -1;
        void* mapped_base = nullptr;
        uintptr_t mapped_address = 0;
        size_t mapped_size = 0;
        std::mutex write_mutex;
    
    public:
        bool initialize(uintptr_t base_addr, size_t size);
        
        bool writeMemory(uintptr_t target_address, uint32_t value);
        bool readMemory(uintptr_t target_address, uint32_t& out_value);

        ~MemoryWriter() {
            if (mapped_base != nullptr) {
                munmap(mapped_base, mapped_size);  // page_size가 아닌 mapped_size 사용
            }
            if (mem_fd != -1) {
                close(mem_fd);
            }
        }
};


class SpiwriteCommand
{
public:
    SpiwriteCommand( Controller::Transport& transport, Controller::CodeGenerator* cgen, Parser::LineParser* parser );
    SpiwriteCommand( const SpiwriteCommand& rhs ) 
        : SpiwriteCommand( rhs.transport_, rhs.code_generator_, rhs.parser_ )
    {}

    // Result Execute( const std::vector<std::string_view>& tokens );
    Result Execute(const std::string& raw_command);

    void fifo_writer(int bus_id, uintptr_t base_addr, MemoryWriter& wr);

    Result parse_binary_commands(const std::vector<uint8_t>& binary_data);
    Result parse_text_commands(const std::vector<std::string_view>& tokens);
    MemoryWriter wr;

private:
    Controller::Transport& transport_;
    Controller::CodeGenerator* code_generator_;
    Parser::LineParser* parser_;
    
};



}
}


#endif
