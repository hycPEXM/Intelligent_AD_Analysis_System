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
};
#endif
