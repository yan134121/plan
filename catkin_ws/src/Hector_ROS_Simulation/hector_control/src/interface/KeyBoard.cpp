#include "../../include/interface/KeyBoard.h"
#include <iostream>

template<typename T>
// 返回两者中较大的一个 即限制了下限为b
inline T max(const T a, const T b){
	return (a > b ? a : b);
}

template<typename T>
// 返回两者中较小的一个 即限制了上限为b
inline T min(const T a, const T b){
	return (a < b ? a : b);
}

KeyBoard::KeyBoard(){
    // 将userCmd userValue 清零
    userCmd = UserCommand::NONE;
    userValue.setZero();
    // 获取 当前终端设置，并将其保存在 _oldSettings 中
    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    // 使用位运算符 清除标志 使用非规范模式并关闭键盘输入的字符回显
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    // 设置 将修改后的设置应用到终端
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );
    // 创建新线程。执行runKeyBoard，并将指向当前类实例的指针作为参数传递。
    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard(){
    // 函数请求线程退出
    pthread_cancel(_tid);
    // 用于等待线程的结束。阻塞调用线程，直到线程退出。NULL指向线程的返回状态的指针表示不关心线程的返回状态
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

/**
 * @brief 检查命令码，运动模式
 * @details 将键盘数值映射成用户命令,将字符映射成命令
 * @param _c KeyBoard类的成员变量，可直接访问
 * @return UserCommand
 * @author wyt
 * @date 2023.11.20
 */
UserCommand KeyBoard::checkCmd(){
    switch (_c){
    // case ' ':
    //     return UserCommand::EXIT;
    case '1':
        return UserCommand::L2_B;
    case '2':
        return UserCommand::L2_A;
    case '3':
        return UserCommand::L2_X;
    case '4':
        return UserCommand::START;
    case '5':
        return UserCommand::L2_Y;
    case '0':
        return UserCommand::L1_X;
    case '9':
        return UserCommand::L1_A;
    case '8':
        return UserCommand::L1_Y;
    case ' ':
        // 空格 将清空命令，使机器人停下来
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

/**
 * @brief KeyBoard 类中的一个成员函数 changeValue 的定义 运动数值
 * @details 限幅+梯度给值，将键盘输入的限制在±1内，只是函数起名有点反了，
 *          两条腿各自四个按键控制水平面两个方向，不分大小写，控制整个身体转弯是。。。
 *          正常逻辑应该是控制整体的移动，而不是单条腿
 *          将运动的数值存起来供上层其他函数调用
 * @param void
 * @return void
 * @author wyt
 * @date 2023.11.20
 */
void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1);
        break;
    default:
        break;
    }
}

/**
 * @brief KeyBoard的线程函数的包装器,用于启动 KeyBoard::run 函数
 * @details 作为线程的入口点通常被要求是静态的或者是全局函数
 *          但是调用的函数名都为run(void *arg),会导致函数乱飞,
 *          所以改为用runKeyBoard(void *arg)进行包装,
 *          而线程库通常允许传递一个 void* 参数给线程入口函数。
 *          因此，为了能够在 runKeyBoard 中调用 run 函数，需要进行类型转换，
 *          将 void* 转换为 KeyBoard* 类型，然后再调用 run 函数。
 *          保持了线程入口点的静态性质，同时通过传递 this 指针给 run 函数，实现了对对象的调用
 * @param 
 * @return 
 * @author wyt
 * @date 2023.11.20
 */
void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
}

/**
 * @brief 键盘输入监测的线程函数
 * @details 该线程函数在一个循环中监测键盘输入，当有输入时进行相应的处理，然后继续等待
 * @param arg 有但没用，相当于没有
 * @return void类型的指针
 * @author wyt
 * @date 2023.11.20
 */
void* KeyBoard::run(void *arg){
    while(1){
        // std::cout << "key test" << std::endl;
        // 清空文件描述符集合是为了确保每次调用 select 之前都处于一个初始的状态
        FD_ZERO(&set);
        // 将标准输入文件描述符添加到文件描述符集合中
        FD_SET( fileno( stdin ), &set );
        // 使用 select 函数监测标准输入是否有可读事件，res 会返回可读事件的数量。
        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            // 读取一个字符，存储在 _c 变量中
            read( fileno( stdin ), &_c, 1 );
            // 调用 checkCmd() 函数判断用户输入的命令，并将结果存储在 userCmd 变量中。
            // 1、2、3、4、5、0、8、9， 不是以上值或空格的时候返回NONE
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                // 只有走动模式下才使用赋值函数
                changeValue();
            // 抛弃 _c，将变量重置为空字符
            _c = '\0';
        }
        // 线程休眠 1000 微秒，避免频繁检查键盘输入 没有输入就1秒检查一次 防止阻塞其他线程
        usleep(1000);
    }
}