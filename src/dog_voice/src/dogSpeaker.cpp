#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "dog_voice/voiceGenerater.hpp"
#include "dog_voice/voicePlayer.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dog_voice_node");

    std::string defaultModelDir;
    try {
        std::string share = ament_index_cpp::get_package_share_directory("dog_voice");
        defaultModelDir = share + "/model";
    } catch (const std::exception &e) {
        RCLCPP_WARN_STREAM(node->get_logger(),
                           "Can't read installed model dir: " << e.what());
        defaultModelDir = "../model";
    }
    std::string modelDir =
        node->declare_parameter<std::string>("model_dir", defaultModelDir);
    std::string refAudioPath = node->declare_parameter<std::string>(
        "ref_audio_path", defaultModelDir + "/ref.wav");
    std::string textTopic =
        node->declare_parameter<std::string>("textTopic", "dog/voice_text");

    Dog8Dof::VoiceGenerater generater(node);
    Dog8Dof::VoicePlayer player;
    try {

        if (!generater.init(modelDir, refAudioPath)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to initialize VoiceGenerater");
            return 1;
        }

        if (!player.init(generater.getSampleRate())) {
            RCLCPP_ERROR(node->get_logger(), "Failed to initialize VoicePlayer");
            return 1;
        }
    } catch (std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
        return 1;
    }

    bool newText = false;
    std::string text;
    auto subCallback = [&](const std::shared_ptr<std_msgs::msg::String> msg) {
        text = msg->data;
        newText = true;
    };
    auto sub = node->create_subscription<std_msgs::msg::String>(
        textTopic, rclcpp::QoS(1), subCallback);

    rclcpp::Rate loopRate(1);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        if (newText) {
            newText = false;
            std::vector<float> audioData = generater.generateAudio(text);
            if (!audioData.empty()) {
                player.play(audioData);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to generate audio");
            }
        }
        loopRate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}