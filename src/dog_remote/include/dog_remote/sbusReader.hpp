#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

namespace Dog8Dof {

    class SbusDecoder {
      public:
        explicit SbusDecoder(const std::string &port = "/dev/ttyUSB0",
                             uint32_t baudrate = 100000);
        ~SbusDecoder();

        uint16_t getChannelValue(size_t channelIndex) const;
        std::array<uint16_t, 16> getAllChannelValues() const;
        bool hasFreshFrame(double timeoutSeconds = 0.1) const;

      private:
        static constexpr size_t sbusFrameSize = 25;
        static constexpr uint8_t sbusFrameHeader = 0x0F;
        static constexpr uint8_t sbusFrameEnd = 0x00;
        static constexpr size_t sbusChannelSize = 16;
        static constexpr size_t filterWindowSize = 5;

        void startAsyncRead();
        void processBuffer();
        bool decodeFrame(const uint8_t *frame);

        void applyFilter();
        uint16_t medianFilter(size_t channelIndex, uint16_t newValue);
        uint16_t hampelFilter(size_t channelIndex, uint16_t newValue);

      private:
        std::string serialPath_;
        uint32_t baudrate_;

        boost::asio::io_context ioContext_;
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
            workGuard_;
        boost::asio::serial_port serialPort_;
        std::thread ioThread_;
        std::atomic<bool> running_{true};

        std::vector<uint8_t> dataBuffer_;
        std::array<uint16_t, sbusChannelSize> channelData_{};
        std::array<std::array<uint16_t, filterWindowSize>, sbusChannelSize>
            filterBuffer_{};
        std::array<size_t, sbusChannelSize> filterBufferIndex_{};

        bool filterEnabled_ = true;
        double madK_ = 3.0;

        mutable std::mutex dataMutex_;
        std::chrono::steady_clock::time_point lastReceiveTime_;
    };

} // namespace Dog8Dof