#pragma once
#include <string>
#include <modbus/modbus.h>
#include <array>
#include <vector>
#include <cmath>

class Yj10
{
private:
    const uint16_t MIN_JOINT_PWM_ = 950;
    const uint16_t MAX_JOINT_PWM_ = 2050;
    const double MIDDLE_JOINT_PWM_ = ((double)MIN_JOINT_PWM_ + (double)MAX_JOINT_PWM_) / 2;
    const double DEG_PER_PWM_ = 180.0 / 1100.0;
    const double RAD_PER_PWM_ = DEG_PER_PWM_ / 180.0 * M_PI;

    double PwmToRad(double pwm) const
    {
        return (pwm - MIDDLE_JOINT_PWM_) * RAD_PER_PWM_;
    }

    double PwmToDeg(double pwm) const
    {
        return (pwm - MIDDLE_JOINT_PWM_) * DEG_PER_PWM_;
    }

    double RadToPwm(double rad) const
    {
        return rad / RAD_PER_PWM_ + MIDDLE_JOINT_PWM_;
    }

    double DegToPwm(double deg) const
    {
        return deg / DEG_PER_PWM_ + MIDDLE_JOINT_PWM_;
    }

public:
    enum class ClamperState
    {
        Stop,
        Close,
        Open,
        Middle,
        Error
    };

    /**
     * @brief yj10 排爆机械臂
     *
     */
    Yj10();
    ~Yj10();

    /**
     * @brief 建立连接
     *
     * @param device the name of the serial port handled by the OS, eg. "/dev/ttyS0" or "/dev/ttyUSB0".
     * On Windows, it's necessary to prepend COM name with "\.\" for COM number greater than 9, eg. "\\.\COM10".
     * See http://msdn.microsoft.com/en-us/library/aa365247(v=vs.85).aspx for details
     *
     * @param device_id 机械臂 id （出厂值为 0x01）
     * @param baud 波特率
     * @param parity 'N' for none, 'E' for even, 'O' for odd
     * @param data_bit The number of bits of data, the allowed values are 5, 6, 7 and 8.
     * @param stop_bit The bits of stop, the allowed values are 1 and 2.
     *
     */
    void Connect(const std::string device, int device_id = 0x01, int baud = 9600, char parity = 'N', int data_bit = 8, int stop_bit = 1);

    /**
     * @brief 关闭连接
     *
     */
    void Close();

    /**
     * @brief 回到初始位姿
     *
     */
    void ResetPose();

    /**
     * @brief 从机械臂读取所有的输入寄存器
     * @note 输入寄存器只读。
     * @note 包括：各个关节的 PWM 最大最小值、夹持器最大电流等参数，不需要连续读取
     */
    void ReadAllInputRegs()
    {
        ReadInputRegisters(0x0, input_regs.size());
    }

    /**
     * @brief 从机械臂读取所有的保持寄存器
     * @note 保持寄存器大部分可读写
     * @note 包括各个关节当前 PWM 值，夹持器操作、状态、电流等参数，是主要控制对象
     */
    void ReadAllHoldingRegs()
    {
        ReadHoldingRegisters(0x0, 12);
    }

    void ReadAllJointsPwm()
    {
        ReadHoldingRegisters(0x0, 5);
    }

    /**
     * @brief 读取夹持器状态
     *
     */
    void ReadClamper()
    {
        ReadHoldingRegisters(0x6, 5);
    }

    /**
     * @brief 写入单个关节位置 PWM
     *
     * @param index 关节 id，范围：[0,4]
     * @param pwm 关节 pwm 值
     * @note 0: 肩关节.偏航；1: 肩关节.俯仰；2：肘关节；3：腕关节.俯仰；4：腕关节.滚转；
     */
    void WriteJoint(int index, uint16_t pwm)
    {
        if (index >= 0 && index <= 4)
        {
            WriteHoldingRegister(index, pwm);
        }
    }

    /**
     * @brief 写入所有关节位置 PWM
     *
     * @param pwms 各个关节的值
     */
    void WriteAllJoints(std::array<uint16_t, 5> pwms);
    void WriteAllJointsRad(std::array<double, 5> rads);
    void WriteAllJointsRad(std::vector<double> rads);

    /**
     * @brief 写入所有关节位置 PWM
     *
     * @param pwm 各个关节的值，数组个数为5
     */
    void WriteAllJoints(uint16_t pwm[]);
    void WriteAllJointsRad(double rad[]);

    /**
     * @brief 给机械爪发指令
     *
     * @param state 期望机械爪到达哪个状态。
     * @note 为下列值之一：(Yj10::ClamperState::) Stop, Close, Open
     */
    void WriteClamperInstruction(ClamperState state);

    /**
     * @brief 设置机械爪闭合电流
     *
     * @param current_mA 毫安
     */
    void WriteClamperClosingCurrent(uint16_t current_mA)
    {
        WriteHoldingRegister(0x8, current_mA);
    }

    /**
     * @brief 一个关节的 PWM 值，读取前需要使用 ReadAllJointsPwm() 更新
     *
     * @param index 范围 [0,4]（index = 5在该型号中没有）
     * @return uint16_t PWM，(500~2500)
     */
    uint16_t Joint(int index) const
    {
        if (index >= 0 && index <= 5)
        {
            return holding_regs.at(index);
        }
        else
        {
            return 0;
        }
    }

    /**
     * @brief 一个关节的 Rad 值，读取前需要使用 ReadAllJointsPwm() 更新
     *
     * @param index 范围 [0,4]（index = 5在该型号中没有）
     */
    double JointRad(int index) const
    {
        return PwmToRad(Joint(index));
    }

    /**
     * @brief 一个关节的 Deg 值，读取前需要使用 ReadAllJointsPwm() 更新
     *
     * @param index 范围 [0,4]（index = 5在该型号中没有）
     */
    double JointDeg(int index) const
    {
        return PwmToDeg(Joint(index));
    }

    /**
     * @brief 各个关节的 PWM 值，读取前需要使用 ReadAllJointsPwm() 更新
     *
     */
    std::array<uint16_t, 5> Joints() const
    {
        std::array<uint16_t, 5> result;
        for (size_t i = 0; i < result.size(); i++)
        {
            result.at(i) = holding_regs.at(i);
        }
        return result;
    }

    /**
     * @brief 各个关节的 Rad 值，读取前需要使用 ReadAllJointsPwm() 更新
     *
     */
    std::array<double, 5> JointsRad() const
    {
        std::array<double, 5> result;
        for (size_t i = 0; i < result.size(); i++)
        {
            result.at(i) = PwmToRad(holding_regs.at(i));
        }
        return result;
    }

    /**
     * @brief 各个关节的 Deg 值，读取前需要使用 ReadAllJointsPwm() 更新
     *
     */
    std::array<double, 5> JointsDeg() const
    {
        std::array<double, 5> result;
        for (size_t i = 0; i < result.size(); i++)
        {
            result.at(i) = PwmToDeg(holding_regs.at(i));
        }
        return result;
    }

    /**
     * @brief 夹持器状态，读取前需要使用 ReadClamper() 更新
     *
     * @return ClamperState
     */
    ClamperState Clamper() const;

    /**
     * @brief 夹持器当前电流，读取前需要使用 ReadClamper() 更新
     *
     * @return uint16_t 电流 (mA)
     */
    uint16_t ClamperCurrent() const
    {
        return holding_regs.at(0xA);
    }

    uint16_t ClamperClosingCurrent() const
    {
        return holding_regs.at(0x8);
    }

    uint16_t ClamperMaxCurrentMA() const
    {
        return input_regs.at(0x1B);
    }

private:
    modbus_t *mb;
    std::array<uint16_t, 35> input_regs;   // 输入寄存器组
    std::array<uint16_t, 12> holding_regs; // 保持寄存器组
    uint16_t write_buf[12];
    const std::array<uint16_t, 5> reset_pwms = {1500, 1100, 1100, 1800, 1500}; // 初始位姿

    void ThrowException();

    /**
     * @brief 读取输入寄存器，从 start_addr 开始连续往后读取 num 个
     *
     * @param start_addr
     * @param num
     * @return const uint16_t* 读到的数据的首地址
     */
    const uint16_t *ReadInputRegisters(int start_addr, int num);

    /**
     * @brief 读取保持寄存器，从 start_addr 开始连续往后读取 num 个
     *
     * @param start_addr
     * @param num
     * @return const uint16_t* 读到的数据的首地址
     */
    const uint16_t *ReadHoldingRegisters(int start_addr, int num);

    /**
     * @brief 写入保持寄存器，从 start_addr 开始连续往后写入 num 个
     *
     * @param start_addr
     * @param num
     * @param data 要写入的数据，数组个数要等于 num
     */
    void WriteHoldingRegisters(int start_addr, int num, const uint16_t *data);

    /**
     * @brief 写入单个保持寄存器
     *
     * @param reg_addr 寄存器地址
     * @param data 要写入的数据，
     */
    void WriteHoldingRegister(int reg_addr, const uint16_t data)
    {
        if (modbus_write_register(mb, reg_addr, data) == -1)
        {
            ThrowException();
        }
    }
};
