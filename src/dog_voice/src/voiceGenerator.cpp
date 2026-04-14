#include "dog_voice/voiceGenerator.hpp"
static void resample_linear(const float *input,
                            int input_len,
                            int input_rate,
                            std::vector<float> &output,
                            int output_rate) {
    double ratio = (double)input_rate / output_rate;
    int output_len = (int)((double)input_len / ratio);
    output.resize(output_len);

    for (int i = 0; i < output_len; ++i) {
        double src_idx = i * ratio;
        int idx0 = (int)src_idx;
        int idx1 = idx0 + 1;
        double frac = src_idx - idx0;

        if (idx1 >= input_len) {
            output[i] = input[input_len - 1];
        } else {
            output[i] = (float)((1.0 - frac) * input[idx0] + frac * input[idx1]);
        }
    }
}

namespace Dog8Dof {
    VoiceGenerater::VoiceGenerater(const rclcpp::Node::SharedPtr node,
                                   const std::string &modelDir,
                                   const std::string &refAudioPath,
                                   const uint8_t threadNum,
                                   bool printInfo)

        : node_(node) {
        std::vector<float> refSamples;
        int refSampleRate;
        if (!qwen3_tts::load_audio_file(refAudioPath, refSamples, refSampleRate)) {
            RCLCPP_WARN(node_->get_logger(), "Failed to read referemce audio");
            throw std::runtime_error("Can't load audio file");
        }

        const int targetRate = 24000;
        if (refSampleRate != targetRate) {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "Resampling audio from " << refSampleRate << " Hz to "
                                                        << targetRate << " Hz...");
            std::vector<float> resampled;
            resample_linear(refSamples.data(),
                            (int)refSamples.size(),
                            refSampleRate,
                            resampled,
                            targetRate);
            refSamples = std::move(resampled);
        }
        // 解析参考音频

        tts_.set_progress_callback([&](int tokens, int max_tokens) -> void {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "Generating: " << tokens << " / " << max_tokens);
        });
        // 设置运行进度提示

        params_.language_id = 2055;    // 设置语言为中文
        params_.n_threads = threadNum; // 设置计算线程数
        params_.print_timing = printInfo;
        params_.print_progress = printInfo;

        if (!tts_.load_models(modelDir)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load models");
            throw std::runtime_error(tts_.get_error());
        } // 加载模型

        if (!tts_.extract_speaker_embedding(
                refSamples.data(), refSamples.size(), embedding_)) {
            RCLCPP_WARN(node_->get_logger(), "Failed to load referance audio");
            throw std::runtime_error(tts_.get_error());
        }
        // 加载源音频并进行克隆，保存音色数据
    }

    std::vector<float> VoiceGenerater::generateAudio(const std::string &text) {
        qwen3_tts::tts_result result;
        result = tts_.synthesize_with_embedding(
            text, embedding_.data(), (int32_t)embedding_.size(), params_);
        if (!result.success) {
            std::vector<float> empty;
            empty.clear();
            return empty;
        } else {
            if (params_.print_timing) {
                RCLCPP_INFO(node_->get_logger(), "\nTiming:");
                RCLCPP_INFO_STREAM(
                    node_->get_logger(),
                    "  Load:      " << (long long)result.t_load_ms << " ms");
                RCLCPP_INFO_STREAM(
                    node_->get_logger(),
                    "  Tokenize:  " << (long long)result.t_tokenize_ms << " ms");
                RCLCPP_INFO_STREAM(
                    node_->get_logger(),
                    "  Encode:    " << (long long)result.t_encode_ms << " ms");
                RCLCPP_INFO_STREAM(
                    node_->get_logger(),
                    "  Generate:  " << (long long)result.t_generate_ms << " ms");
                RCLCPP_INFO_STREAM(
                    node_->get_logger(),
                    "  Decode:    " << (long long)result.t_decode_ms << " ms");
                RCLCPP_INFO_STREAM(
                    node_->get_logger(),
                    "  Total:     " << (long long)result.t_total_ms << " ms");
            }
            return result.audio;
        }
    }
} // namespace Dog8Dof