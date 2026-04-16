#include "dog_voice/voicePlayer.hpp"
#include <cstring>

namespace Dog8Dof {

    VoicePlayer::VoicePlayer() {
        SDL_Init(SDL_INIT_AUDIO);
    }

    VoicePlayer::~VoicePlayer() {
        stop();
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
    }

    bool VoicePlayer::init(int sampleRate, int channels) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (initialized_)
            return true;
        sampleRate_ = sampleRate;
        channels_ = channels;
        SDL_AudioSpec desired{};
        desired.freq = sampleRate_;
        desired.format = AUDIO_F32SYS;
        desired.channels = static_cast<Uint8>(channels_);
        desired.samples = 4096;
        desired.callback = nullptr;
        device_ = SDL_OpenAudioDevice(
            nullptr, 0, &desired, &spec_, SDL_AUDIO_ALLOW_FORMAT_CHANGE);
        if (device_ == 0)
            return false;
        SDL_PauseAudioDevice(device_, 0);
        initialized_ = true;
        return true;
    }

    bool VoicePlayer::isInitialized() const {
        return initialized_;
    }

    bool VoicePlayer::play(const float *data, std::size_t frames) {
        if (!data || frames == 0)
            return false;
        std::lock_guard<std::mutex> lk(mutex_);
        if (!initialized_)
            return false;
        std::size_t samples = frames * static_cast<std::size_t>(channels_);
        std::size_t bytes = samples * sizeof(float);
        int rc = SDL_QueueAudio(device_, data, static_cast<Uint32>(bytes));
        return rc == 0;
    }

    bool VoicePlayer::play(const std::vector<float> &data) {
        if (data.empty())
            return false;
        if (channels_ <= 0)
            return false;
        std::size_t frames = data.size() / static_cast<std::size_t>(channels_);
        return play(data.data(), frames);
    }

    void VoicePlayer::flush() {
        std::lock_guard<std::mutex> lk(mutex_);
        if (initialized_)
            SDL_ClearQueuedAudio(device_);
    }

    void VoicePlayer::stop() {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!initialized_)
            return;
        SDL_PauseAudioDevice(device_, 1);
        SDL_CloseAudioDevice(device_);
        device_ = 0;
        initialized_ = false;
    }

} // namespace Dog8Dof
