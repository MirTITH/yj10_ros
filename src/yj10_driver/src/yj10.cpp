#include "yj10.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <thread>

// #define DEBUG

using namespace std;

Yj10::ClamperState Yj10::Clamper() const
{
    switch (holding_regs.at(0x07))
    {
    case 0:
        return ClamperState::Middle;
        break;
    case 1:
        return ClamperState::Close;
        break;
    case 2:
        return ClamperState::Open;
        break;

    default:
        return ClamperState::Error;
        break;
    }
}

void Yj10::ThrowException()
{
    auto str_err = modbus_strerror(errno);
    throw runtime_error(str_err);
}

const uint16_t *Yj10::ReadInputRegisters(int start_addr, int num)
{
    if (modbus_read_input_registers(mb, start_addr, num, input_regs.data() + start_addr) == -1)
    {
        ThrowException();
    }
    return input_regs.data() + start_addr;
}

const uint16_t *Yj10::ReadHoldingRegisters(int start_addr, int num)
{
    if (modbus_read_registers(mb, start_addr, num, holding_regs.data() + start_addr) == -1)
    {
        ThrowException();
    }
    return holding_regs.data() + start_addr;
}

void Yj10::WriteHoldingRegisters(int start_addr, int num, const uint16_t *data)
{
    if (modbus_write_registers(mb, start_addr, num, data) == -1)
    {
        ThrowException();
    }
}

Yj10::Yj10()
{
    mb = nullptr;
    input_regs.fill(0);
    holding_regs.fill(0);
    memset(write_buf, 0, sizeof(write_buf));
}

Yj10::~Yj10()
{
    if (mb != nullptr)
    {
        modbus_free(mb);
    }
}

void Yj10::Connect(const std::string device, int device_id, int baud, char parity, int data_bit, int stop_bit)
{
    mb = modbus_new_rtu(device.c_str(), baud, parity, data_bit, stop_bit);

    if (mb == nullptr)
    {
        ThrowException();
    }

    // 设置超时时间
    modbus_set_response_timeout(mb, 0, 100);
    // modbus_set_indication_timeout(mb, 3, 0);
    // modbus_set_byte_timeout(mb, 3, 0);

#ifdef DEBUG
    modbus_set_debug(mb, true);
#endif // DEBUG

    modbus_set_slave(mb, device_id);
    if (modbus_connect(mb) != 0)
    {
        ThrowException();
    }
}

void Yj10::Close()
{
    modbus_close(mb);
}

void Yj10::ResetPose()
{
    bool has_retried = false;
    while (true)
    {
        try
        {
            WriteAllJoints(reset_pwms);
            if (has_retried == true)
            {
                cout << "ResetPose success" << endl;
            }

            break;
        }
        catch (const std::exception &e)
        {
            cerr << "ResetPose failed: " << e.what() << ". Retrying..." << endl;
            has_retried = true;
            this_thread::sleep_for(0.5s);
        }
    }
}

void Yj10::WriteAllJoints(uint16_t pwm[])
{
    for (int i = 0; i < 5; i++)
    {
        write_buf[i] = pwm[i];
    }

    WriteHoldingRegisters(0, 5, write_buf);
}

void Yj10::WriteAllJointsRad(double rad[])
{
    std::array<uint16_t, 5> pwms;
    for (size_t i = 0; i < pwms.size(); i++)
    {
        pwms.at(i) = RadToPwm(rad[i]);
    }
    WriteAllJoints(pwms);
}

void Yj10::WriteClamperInstruction(ClamperState state)
{
    uint16_t data;
    switch (state)
    {
    case ClamperState::Stop:
        data = 0;
        break;
    case ClamperState::Close:
        data = 1;
        break;
    case ClamperState::Open:
        data = 2;
        break;

    default:
        data = 0;
        break;
    }
    WriteHoldingRegister(0x6, data);
}

void Yj10::WriteAllJoints(std::array<uint16_t, 5> pwms)
{
    for (size_t i = 0; i < pwms.size(); i++)
    {
        write_buf[i] = pwms.at(i);
    }

    WriteHoldingRegisters(0, pwms.size(), write_buf);
}

void Yj10::WriteAllJointsRad(std::array<double, 5> rads)
{
    std::array<uint16_t, 5> pwms;
    for (size_t i = 0; i < pwms.size(); i++)
    {
        pwms.at(i) = RadToPwm(rads.at(i));
    }
    WriteAllJoints(pwms);
}
