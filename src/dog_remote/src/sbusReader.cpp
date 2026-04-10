#include "dog_remote/sbusReader.hpp"
#include "dog_remote/serialConfig.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace Dog8Dof {

    SbusReader::SbusReader(const std::string &port, uint32_t baudrate)
        : serialPath_(port), baudrate_(baudrate),
          workGuard_(boost::asio::make_work_guard(ioContext_)), serialPort_(ioContext_) {

        try {
            serialPort_.open(serialPath_);
            setBaudrate();
        } catch (const std::exception &e) {
            throw std::runtime_error(std::string("SBUS serial init failed: ") + e.what());
        }

        lastReceiveTime_ = std::chrono::steady_clock::now();

        startAsyncRead();
        ioThread_ = std::thread([this]() { ioContext_.run(); });
    }

    SbusReader::~SbusReader() {
        running_.store(false);

        try {
            if (serialPort_.is_open()) {
                serialPort_.cancel();
                serialPort_.close();
            }
        } catch (const std::exception &e) {
            std::cerr << "close SBUS serial failed: " << e.what() << std::endl;
        }

        workGuard_.reset();
        ioContext_.stop();

        if (ioThread_.joinable()) {
            ioThread_.join();
        }
    }

    void SbusReader::startAsyncRead() {
        if (!running_.load()) {
            return;
        }

        boost::asio::async_read(
            serialPort_,
            boost::asio::dynamic_buffer(dataBuffer_),
            boost::asio::transfer_at_least(sbusFrameSize),
            [this](const boost::system::error_code &ec, std::size_t bytesRead) {
                if (!running_.load()) {
                    return;
                }

                if (ec) {
                    if (ec != boost::asio::error::operation_aborted) {
                        std::cerr << "async read failed: " << ec.message() << std::endl;
                    }
                    if (running_.load()) {
                        startAsyncRead();
                    }
                    return;
                }

                (void)bytesRead;
                processBuffer();
                startAsyncRead();
            });
    }

    void SbusReader::processBuffer() {

        while (dataBuffer_.size() >= sbusFrameSize) {
            if (dataBuffer_[0] != sbusFrameHeader) {
                dataBuffer_.erase(dataBuffer_.begin());
                continue;
            }

            if (dataBuffer_[sbusFrameSize - 1] != sbusFrameEnd) {
                dataBuffer_.erase(dataBuffer_.begin());
                continue;
            }

            if (decodeFrame(dataBuffer_.data())) {
                std::lock_guard<std::mutex> lock(dataMutex_);
                lastReceiveTime_ = std::chrono::steady_clock::now();
                if (filterEnabled_) {
                    applyFilter();
                }
            }

            dataBuffer_.erase(
                dataBuffer_.begin(),
                dataBuffer_.begin() + static_cast<std::ptrdiff_t>(sbusFrameSize));
        }
    }

    bool SbusReader::decodeFrame(const uint8_t *d) {
        channelData_[0] = ((d[1] | d[2] << 8) & 0x07FF);
        channelData_[1] = ((d[2] >> 3 | d[3] << 5) & 0x07FF);
        channelData_[2] = ((d[3] >> 6 | d[4] << 2 | d[5] << 10) & 0x07FF);
        channelData_[3] = ((d[5] >> 1 | d[6] << 7) & 0x07FF);
        channelData_[4] = ((d[6] >> 4 | d[7] << 4) & 0x07FF);
        channelData_[5] = ((d[7] >> 7 | d[8] << 1 | d[9] << 9) & 0x07FF);
        channelData_[6] = ((d[9] >> 2 | d[10] << 6) & 0x07FF);
        channelData_[7] = ((d[10] >> 5 | d[11] << 3) & 0x07FF);
        channelData_[8] = ((d[12] | d[13] << 8) & 0x07FF);
        channelData_[9] = ((d[13] >> 3 | d[14] << 5) & 0x07FF);
        channelData_[10] = ((d[14] >> 6 | d[15] << 2 | d[16] << 10) & 0x07FF);
        channelData_[11] = ((d[16] >> 1 | d[17] << 7) & 0x07FF);
        channelData_[12] = ((d[17] >> 4 | d[18] << 4) & 0x07FF);
        channelData_[13] = ((d[18] >> 7 | d[19] << 1 | d[20] << 9) & 0x07FF);
        channelData_[14] = ((d[20] >> 2 | d[21] << 6) & 0x07FF);
        channelData_[15] = ((d[21] >> 5 | d[22] << 3) & 0x07FF);
        return true;
    }

    void SbusReader::applyFilter() {
        for (size_t ch = 0; ch < sbusChannelSize; ++ch) {
            const uint16_t medianValue = medianFilter(ch, channelData_[ch]);
            channelData_[ch] = hampelFilter(ch, medianValue);
        }
    }

    uint16_t SbusReader::medianFilter(size_t channelIndex, uint16_t newValue) {
        filterBuffer_[channelIndex][filterBufferIndex_[channelIndex]] = newValue;
        filterBufferIndex_[channelIndex] =
            (filterBufferIndex_[channelIndex] + 1) % filterWindowSize;

        auto sorted = filterBuffer_[channelIndex];
        std::sort(sorted.begin(), sorted.end());
        return sorted[filterWindowSize / 2];
    }

    uint16_t SbusReader::hampelFilter(size_t channelIndex, uint16_t newValue) {
        auto window = filterBuffer_[channelIndex];
        auto sorted = window;
        std::sort(sorted.begin(), sorted.end());
        const double median = static_cast<double>(sorted[filterWindowSize / 2]);

        std::array<double, filterWindowSize> deviations{};
        for (size_t i = 0; i < filterWindowSize; ++i) {
            deviations[i] = std::abs(static_cast<double>(window[i]) - median);
        }
        std::sort(deviations.begin(), deviations.end());
        const double mad = deviations[filterWindowSize / 2];

        if (mad <= 1e-9) {
            return newValue;
        }

        return (std::abs(static_cast<double>(newValue) - median) > madK_ * mad)
                   ? static_cast<uint16_t>(median)
                   : newValue;
    }

    uint16_t SbusReader::getChannelValue(size_t channelIndex) const {
        if (channelIndex >= sbusChannelSize) {
            throw std::out_of_range("channelIndex out of range");
        }

        std::lock_guard<std::mutex> lock(dataMutex_);
        return channelData_[channelIndex];
    }

    std::array<uint16_t, 16> SbusReader::getAllChannelValues() const {
        std::lock_guard<std::mutex> lock(dataMutex_);
        return channelData_;
    }

    bool SbusReader::hasFreshFrame(double timeoutSeconds) const {
        std::lock_guard<std::mutex> lock(dataMutex_);
        const auto elapsed = std::chrono::steady_clock::now() - lastReceiveTime_;
        return std::chrono::duration<double>(elapsed).count() <= timeoutSeconds;
    }

    void SbusReader::setBaudrate() {
        const int fd = serialPort_.native_handle();
        std::string err;
        const bool ok = configureSerialFdLinux(
            fd,
            baudrate_,
            false, // false=8N1(对齐 pyserial 默认), true=8E2(标准 SBUS)
            0,     // VMIN
            1,     // VTIME, 0.1s
            &err);

        if (!ok) {
            throw std::runtime_error(err);
        }
    }

} // namespace Dog8Dof
