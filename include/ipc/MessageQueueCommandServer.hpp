/**
 * @file MessageQueueCommandServer.hpp
 * @brief Command Queue Server and Client for Inter-Process Communication
 */

#ifndef MESSAGE_QUEUE_COMMAND_SERVER_HPP
#define MESSAGE_QUEUE_COMMAND_SERVER_HPP

#include "common/MotorTypes.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace MotorHub {

class CommandQueueServer {
public:
    static CommandQueueServer& getInstance() {
        static CommandQueueServer instance;
        return instance;
    }

    void pushCommand(const CommandPacket& cmd) {
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            cmdQueue_.push(cmd);
        }
        cv_.notify_one();
    }

    bool popCommand(CommandPacket& outCmd, int timeoutMs = 10) {
        std::unique_lock<std::mutex> lock(queueMutex_);
        if (cmdQueue_.empty()) {
            if (timeoutMs > 0) {
                cv_.wait_for(lock, std::chrono::milliseconds(timeoutMs), [this]() {
                    return !cmdQueue_.empty();
                });
            }
        }
        if (cmdQueue_.empty()) return false;
        outCmd = cmdQueue_.front();
        cmdQueue_.pop();
        return true;
    }

    bool hasPendingCommands() const {
        std::lock_guard<std::mutex> lock(queueMutex_);
        return !cmdQueue_.empty();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(queueMutex_);
        while (!cmdQueue_.empty()) cmdQueue_.pop();
    }

private:
    CommandQueueServer() = default;
    ~CommandQueueServer() = default;
    CommandQueueServer(const CommandQueueServer&) = delete;
    CommandQueueServer& operator=(const CommandQueueServer&) = delete;

    mutable std::mutex queueMutex_;
    std::condition_variable cv_;
    std::queue<CommandPacket> cmdQueue_;
};

} // namespace MotorHub

#endif // MESSAGE_QUEUE_COMMAND_SERVER_HPP
