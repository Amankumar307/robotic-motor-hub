/**
 * @file SharedMemoryTelemetry.cpp
 * @brief High-Speed Shared Memory Telemetry Buffer Implementation
 */

#include "ipc/SharedMemoryTelemetry.hpp"
#include "common/Logger.hpp"
#include <cstring>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

namespace MotorHub {

SharedMemoryPublisher::SharedMemoryPublisher(std::string shmName)
    : shmName_(std::move(shmName)) {}

SharedMemoryPublisher::~SharedMemoryPublisher() {
    close();
}

bool SharedMemoryPublisher::initialize() {
    constexpr size_t shmSize = sizeof(SystemTelemetryPacket);

#if defined(__linux__)
    shmFd_ = ::shm_open(shmName_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shmFd_ < 0) {
        LOG_ERROR("SHM_Pub", "Failed to open POSIX shared memory segment: " + shmName_);
        return false;
    }

    if (::ftruncate(shmFd_, shmSize) < 0) {
        LOG_ERROR("SHM_Pub", "Failed to ftruncate SHM");
        ::close(shmFd_);
        shmFd_ = -1;
        return false;
    }

    void* addr = ::mmap(nullptr, shmSize, PROT_READ | PROT_WRITE, MAP_SHARED, shmFd_, 0);
    if (addr == MAP_FAILED) {
        LOG_ERROR("SHM_Pub", "Failed to mmap SHM");
        ::close(shmFd_);
        shmFd_ = -1;
        return false;
    }

    sharedBuffer_ = static_cast<SystemTelemetryPacket*>(addr);
    std::memset(sharedBuffer_, 0, shmSize);
    LOG_INFO("SHM_Pub", "POSIX Shared Memory Telemetry Publisher active on " + shmName_);
    return true;

#elif defined(_WIN32) || defined(_WIN64)
    std::string winMapName = "Local\\" + shmName_;
    HANDLE hMapFile = CreateFileMappingA(
        INVALID_HANDLE_VALUE,
        NULL,
        PAGE_READWRITE,
        0,
        static_cast<DWORD>(shmSize),
        winMapName.c_str()
    );

    if (hMapFile == NULL) {
        LOG_ERROR("SHM_Pub", "CreateFileMapping failed (Windows Error: " + std::to_string(GetLastError()) + ")");
        return false;
    }

    mapHandle_ = hMapFile;
    void* addr = MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, shmSize);
    if (addr == NULL) {
        CloseHandle(hMapFile);
        mapHandle_ = nullptr;
        return false;
    }

    sharedBuffer_ = static_cast<SystemTelemetryPacket*>(addr);
    std::memset(sharedBuffer_, 0, shmSize);
    LOG_INFO("SHM_Pub", "Shared Memory Telemetry Publisher active on " + winMapName);
    return true;
#endif
}

void SharedMemoryPublisher::publish(const SystemTelemetryPacket& packet) {
    if (!sharedBuffer_) return;
    // Copy packet into shared memory segment
    std::memcpy(sharedBuffer_, &packet, sizeof(SystemTelemetryPacket));
}

void SharedMemoryPublisher::close() {
#if defined(__linux__)
    if (sharedBuffer_) {
        ::munmap(sharedBuffer_, sizeof(SystemTelemetryPacket));
        sharedBuffer_ = nullptr;
    }
    if (shmFd_ >= 0) {
        ::close(shmFd_);
        ::shm_unlink(shmName_.c_str());
        shmFd_ = -1;
    }
#elif defined(_WIN32) || defined(_WIN64)
    if (sharedBuffer_) {
        UnmapViewOfFile(sharedBuffer_);
        sharedBuffer_ = nullptr;
    }
    if (mapHandle_) {
        CloseHandle(static_cast<HANDLE>(mapHandle_));
        mapHandle_ = nullptr;
    }
#endif
}

// Subscriber Implementation
SharedMemorySubscriber::SharedMemorySubscriber(std::string shmName)
    : shmName_(std::move(shmName)) {}

SharedMemorySubscriber::~SharedMemorySubscriber() {
    close();
}

bool SharedMemorySubscriber::initialize() {
    constexpr size_t shmSize = sizeof(SystemTelemetryPacket);

#if defined(__linux__)
    shmFd_ = ::shm_open(shmName_.c_str(), O_RDONLY, 0666);
    if (shmFd_ < 0) {
        return false;
    }

    void* addr = ::mmap(nullptr, shmSize, PROT_READ, MAP_SHARED, shmFd_, 0);
    if (addr == MAP_FAILED) {
        ::close(shmFd_);
        shmFd_ = -1;
        return false;
    }

    sharedBuffer_ = static_cast<const SystemTelemetryPacket*>(addr);
    return true;

#elif defined(_WIN32) || defined(_WIN64)
    std::string winMapName = "Local\\" + shmName_;
    HANDLE hMapFile = OpenFileMappingA(FILE_MAP_READ, FALSE, winMapName.c_str());
    if (hMapFile == NULL) {
        return false;
    }

    mapHandle_ = hMapFile;
    void* addr = MapViewOfFile(hMapFile, FILE_MAP_READ, 0, 0, shmSize);
    if (addr == NULL) {
        CloseHandle(hMapFile);
        mapHandle_ = nullptr;
        return false;
    }

    sharedBuffer_ = static_cast<const SystemTelemetryPacket*>(addr);
    return true;
#endif
}

bool SharedMemorySubscriber::readLatest(SystemTelemetryPacket& outPacket) {
    if (!sharedBuffer_) return false;
    std::memcpy(&outPacket, sharedBuffer_, sizeof(SystemTelemetryPacket));
    lastSequence_ = outPacket.sequenceNumber;
    return true;
}

void SharedMemorySubscriber::close() {
#if defined(__linux__)
    if (sharedBuffer_) {
        ::munmap(const_cast<SystemTelemetryPacket*>(sharedBuffer_), sizeof(SystemTelemetryPacket));
        sharedBuffer_ = nullptr;
    }
    if (shmFd_ >= 0) {
        ::close(shmFd_);
        shmFd_ = -1;
    }
#elif defined(_WIN32) || defined(_WIN64)
    if (sharedBuffer_) {
        UnmapViewOfFile(sharedBuffer_);
        sharedBuffer_ = nullptr;
    }
    if (mapHandle_) {
        CloseHandle(static_cast<HANDLE>(mapHandle_));
        mapHandle_ = nullptr;
    }
#endif
}

} // namespace MotorHub
