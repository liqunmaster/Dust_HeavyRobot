#pragma once

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

// 基于自旋锁的通用主题通道，保存最新一条消息供读取
template <typename MsgType>
class ZbusChannel
{
    public:
    /**
     * @brief 初始化通道，清空消息标志并复位锁
    */
    void init()
    {
        lock_ = {};
        has_msg_ = false;
    }

    /**
     * @brief 发布一条新消息并覆盖上一条消息
     *
     * @param msg 待发布的消息
    */
    void publish(const MsgType &msg)
    {
        k_spinlock_key_t key = k_spin_lock(&lock_);
        last_msg_ = msg;
        has_msg_ = true;
        k_spin_unlock(&lock_, key);
    }

    /**
     * @brief 读取最新一条消息
     *
     * @param msg 消息输出参数
     * @return 成功读取返回 0，暂无消息返回 -ENOMSG
    */
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