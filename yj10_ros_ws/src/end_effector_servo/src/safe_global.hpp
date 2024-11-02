#pragma once

#include <mutex>

// 用于保护全局变量线程安全
template <typename T>
class SafeGlobal
{
public:
    SafeGlobal() {}
    SafeGlobal(const T &value) : data_(value) {}

    T get()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_;
    }

    void set(const T &value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = value;
    }

    void lock()
    {
        mutex_.lock();
    }

    void unlock()
    {
        mutex_.unlock();
    }

    T &data()
    {
        return data_;
    }

private:
    T data_;
    std::mutex mutex_;
};