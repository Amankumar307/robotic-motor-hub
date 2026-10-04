/**
 * @file SharedMemoryTelemetry.hpp
 * @brief High-Speed POSIX / Cross-Platform Shared Memory Telemetry Buffer
 */

#ifndef SHARED_MEMORY_TELEMETRY_HPP
#define SHARED_MEMORY_TELEMETRY_HPP

#include "common/MotorTypes.hpp"
#include <string>
#include <atomic>

namespace MotorHub {

class SharedMemoryPublisher {
public:
    explicit SharedMemoryPublisher(std::string shmName = "/motor_hub_telemetry");
    ~SharedMemoryPublisher();

    bool initialize();
    void publish(const SystemTelemetryPacket& packet);
    void close();

private:
    std::string shmName_;
    SystemTelemetryPacket* sharedBuffer_{nullptr};
    int shmFd_{-1};
#if defined(_WIN32) || defined(_WIN64)
    void* mapHandle_{nullptr};
#endif
};

class SharedMemorySubscriber {
public:
    explicit SharedMemorySubscriber(std::string shmName = "/motor_hub_telemetry");
    ~SharedMemorySubscriber();

    bool initialize();
    bool readLatest(SystemTelemetryPacket& outPacket);
    void close();

private:
    std::string shmName_;
    const SystemTelemetryPacket* sharedBuffer_{nullptr};
    int shmFd_{-1};
#if defined(_WIN32) || defined(_WIN64)
    void* mapHandle_{nullptr};
#endif
    uint64_t lastSequence_{0};
};

} // namespace MotorHub

#endif // SHARED_MEMORY_TELEMETRY_HPP
