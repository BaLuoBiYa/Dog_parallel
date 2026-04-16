#pragma once
#include <SDL2/SDL.h>
#include <cstddef>
#include <mutex>
#include <vector>

namespace Dog8Dof {

    class VoicePlayer {
      public:
        VoicePlayer();
        ~VoicePlayer();

        bool init(int sampleRate = 24000, int channels = 1);
        bool isInitialized() const;

        bool play(const float *data, std::size_t frames);
        bool play(const std::vector<float> &data);

        void flush();
        void stop();

      private:
        SDL_AudioDeviceID device_ = 0;
        SDL_AudioSpec spec_{};
        int sampleRate_ = 24000;
        int channels_ = 1;
        bool initialized_ = false;
        std::mutex mutex_;
    };

} // namespace Dog8Dof