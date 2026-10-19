#pragma once

#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

template <typename MsgType>
class ZbusChannel
{
    public:

    void Init()
    {
        lock_ = {};
        has_msg_ = false;
        sequence_ = 0;
        last_msg_ = {};
    }

    void Publish(const MsgType &msg)
    {
        k_spinlock_key_t key = k_spin_lock(&lock_);
        last_msg_ = msg;
        ++sequence_;
        has_msg_ = true;
        k_spin_unlock(&lock_, key);
    }

    int Read(MsgType &msg)
    {
        uint32_t sequence;
        return Read(msg, sequence);
    }

    int Read(MsgType &msg, uint32_t &sequence)
    {
        k_spinlock_key_t key = k_spin_lock(&lock_);
        if (has_msg_) {
            msg = last_msg_;
            sequence = sequence_;
            k_spin_unlock(&lock_, key);
            return 0;
        }
        k_spin_unlock(&lock_, key);
        return -ENOMSG;
    }

    private:
    struct k_spinlock lock_{};

    MsgType last_msg_{};

    uint32_t sequence_ = 0;

    bool has_msg_ = false;
};
