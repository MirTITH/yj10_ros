#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class KeyboardAsync
{
private:
    struct termios oldt, newt;
    int flags;

public:
    KeyboardAsync()
    {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    /**
     * @brief 异步读取键盘
     *
     * @return int 如果没读到，返回 EOF
     */
    int Read()
    {
        return getchar();
    }

    ~KeyboardAsync()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
};
