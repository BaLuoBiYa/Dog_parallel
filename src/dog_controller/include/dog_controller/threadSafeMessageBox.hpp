#pragma once

#include <mutex>
#include <optional>

namespace Dog8Dof {

    template <typename T>
    class ThreadSafeMessageBox {
      public:
        bool try_set(const T &value) {
            std::lock_guard<std::mutex> lock(mutex_);
            value_ = value;
            return true;
        }

        std::optional<T> try_get() {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!value_.has_value()) {
                return std::nullopt;
            }

            std::optional<T> out = value_;
            value_.reset();
            return out;
        }

      private:
        std::mutex mutex_;
        std::optional<T> value_;
    };

} // namespace Dog8Dof