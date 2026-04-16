#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "qwen3_tts/include/qwen3_tts.h"

namespace Dog8Dof {
    class VoiceGenerater {
      public:
        VoiceGenerater(const rclcpp::Node::SharedPtr node,
                       const uint8_t threadNum = 4,
                       bool printInfo = true);

        bool init(const std::string &modelDir, const std::string &refAudioPath);
        size_t getSampleRate() {
            return 24000;
        }
        std::vector<float> generateAudio(const std::string &text);

      private:
        bool isInit_ = false;

        const rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sentenceReceiver_;

        std::string newSentence_;
        std::vector<float> embedding_;

        qwen3_tts::Qwen3TTS tts_;
        qwen3_tts::tts_params params_;
    };
} // namespace Dog8Dof
