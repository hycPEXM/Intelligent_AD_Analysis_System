// ******************************************************
// Author       : Joey Qiu
// Last modified: 2023-06-02 16:56
// Email        : 2650982628@qq.com
// Filename     : Function.hpp
// Description  : 
// ******************************************************
#ifndef FUNCTION_HPP
#define FUNCTION_HPP
#include <cstring>
#include <iostream>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <vector>
#include <sstream>
#include <fstream>


class Function{
public:
    Function() = default;

    static bool kbhit(){
        termios term;
        tcgetattr(0, &term);

        termios term2 = term;
        term2.c_lflag &= ~ICANON;
        tcsetattr(0, TCSANOW, &term2);

        int byteswaiting;
        ioctl(0, FIONREAD, &byteswaiting);

        tcsetattr(0, TCSANOW, &term);

        return byteswaiting > 0;
    }

    static std::vector<std::string> splitString(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::istringstream iss(str);
        std::string token;
        while (std::getline(iss, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    static void* readFileToBuffer(const std::string& filename, uint32_t& length) {
        std::ifstream file(filename, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return nullptr;
        }

        length = file.tellg(); // 获取文件长度
        file.seekg(0, std::ios::beg); // 将文件指针移回文件开头

        void* buffer = new char[length];
        file.read(static_cast<char*>(buffer), length); // 读取文件内容到缓冲区

        file.close();

        return buffer;
    }
};
#endif
