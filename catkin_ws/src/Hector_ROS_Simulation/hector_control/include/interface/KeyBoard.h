#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "CmdPanel.h"
#include "../common/cppTypes.h"

/**
 * @brief 类:KeyBoard 的定义，继承自 CmdPanel 类的公共部分
 * @details 
 * @author wyt
 * @date 2023.11.20
 */
class KeyBoard : public CmdPanel{
public:
    KeyBoard();
    ~KeyBoard();
private:
    static void *runKeyBoard(void *arg);
    void *run(void *arg); //总运行函数
    UserCommand checkCmd(); //检查命令码 将键盘数值映射成用户命令
    void changeValue(); // 赋值函数

    pthread_t _tid; // 存储获取的线程ID号
    float sensitivityLeft = 0.025; // 左腿灵敏度
    float sensitivityRight = 0.025; // 右腿灵敏度
    struct termios _oldSettings, _newSettings;
    fd_set set; // 文件描述符集合
    int res; // 选择函数的返回值
    char _c; // 键盘输入字符
};

#endif  // KEYBOARD_H