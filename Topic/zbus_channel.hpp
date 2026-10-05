#pragma once

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

template <typename MsgType>
class ZbusChannel
{
    public:
    void init()
    {
        lock_ = {};
        has_msg_ = false;
    }

    int publish(const MsgType &msg)
    {
        k_spinlock_key_t key = k_spin_lock(&lock_);
        last_msg_ = msg;
        has_msg_ = true;
        k_spin_unlock(&lock_, key);
        return 0;
    }

    int read(MsgType &msg)
    {
        k_spinlock_key_t key = k_spin_lock(&lock_);
        if (has_msg_) {
            msg = last_msg_;
            k_spin_unlock(&lock_, key);
            return 0;
        }
        k_spin_unlock(&lock_, key);
        return -ENOMSG;
    }

    private:
    struct k_spinlock lock_;

    MsgType last_msg_;

    bool has_msg_ = false;
};
