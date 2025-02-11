//---------------------------------------------------------------------------------------------------------------------
//  LUCAS: Lightweight framework for UAV Control And Supervision
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2024 CATEC (Advanced Centre for Aerospace Technologies)
//---------------------------------------------------------------------------------------------------------------------
// This program is free software: you may redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
// version.
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with this program. If not, see
// https://www.gnu.org/licenses/.
//---------------------------------------------------------------------------------------------------------------------

#pragma once

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <filesystem>
#include <memory>

#define LOG_DEFAULT_LOGGER_NAME "catec_logger"
// #define DISABLE_SPDLOG_ON_RELEASE_BUILD /*Disable spdlog for release builds*/
class LogManager
{
  public:
    static LogManager& instance()
    {
        static LogManager instance;
        return instance;
    }

    void initialize(
            const std::string&               file_name,
            const spdlog::level::level_enum& console_level,
            const spdlog::level::level_enum& file_level,
            const std::size_t                max_files = 50)
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e][%l] %v%$");
        console_sink->set_level(console_level);

        /// \note. Create a file rotating logger with and 100 rotated files
        std::size_t max_size  = 1048576 * 10; // 10 MB size max
        auto        file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(file_name, max_size, max_files);
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e][%l] %v");
        file_sink->set_level(file_level);

        std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
        _logger = std::make_shared<spdlog::logger>(LOG_DEFAULT_LOGGER_NAME, sinks.begin(), sinks.end());
        _logger->set_level(spdlog::level::trace);
        _logger->flush_on(spdlog::level::trace);

        spdlog::register_logger(_logger);

        if (spdlog::get(LOG_DEFAULT_LOGGER_NAME) == nullptr)
            SPDLOG_ERROR("Error creating {} instance", LOG_DEFAULT_LOGGER_NAME);

        if (max_files > 0)
            _cleanup_old_logs(std::filesystem::path(file_name).parent_path(), max_files);
    }

    spdlog::logger& get_logger() { return *_logger; }

    void shutdown() { spdlog::shutdown(); }

  private:
    LogManager()                  = default;
    LogManager(const LogManager&) = delete;
    void _cleanup_old_logs(const std::string& directory, const std::size_t max_files)
    {
        std::vector<std::filesystem::directory_entry> files;

        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".log") {
                files.push_back(entry);
            }
        }

        std::sort(
                files.begin(),
                files.end(),
                [](const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b) {
                    return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
                });

        while (files.size() > max_files) {
            std::filesystem::remove(files.front());
            files.erase(files.begin());
        }
    }
    LogManager&                     operator=(const LogManager&) = delete;
    std::shared_ptr<spdlog::logger> _logger;
    std::size_t                     _max_files{0};

    ~LogManager() = default;
};

#define LOG_TRACE_THROTTLE(frequency, ...)                                                                \
    do {                                                                                                  \
        static std::chrono::steady_clock::duration throttle_duration                                      \
                = std::chrono::duration_cast<std::chrono::steady_clock::duration>(                        \
                        std::chrono::duration<double>(1.0 / frequency));                                  \
        static std::chrono::time_point<std::chrono::steady_clock> last_log_time{};                        \
        auto                                                      now = std::chrono::steady_clock::now(); \
        if (now - last_log_time >= throttle_duration) {                                                   \
            last_log_time = now;                                                                          \
            LOG_TRACE(__VA_ARGS__);                                                                       \
        }                                                                                                 \
    } while (0)

#define LOG_DEBUG_THROTTLE(frequency, ...)                                                                \
    do {                                                                                                  \
        static std::chrono::steady_clock::duration throttle_duration                                      \
                = std::chrono::duration_cast<std::chrono::steady_clock::duration>(                        \
                        std::chrono::duration<double>(1.0 / frequency));                                  \
        static std::chrono::time_point<std::chrono::steady_clock> last_log_time{};                        \
        auto                                                      now = std::chrono::steady_clock::now(); \
        if (now - last_log_time >= throttle_duration) {                                                   \
            last_log_time = now;                                                                          \
            LOG_DEBUG(__VA_ARGS__);                                                                       \
        }                                                                                                 \
    } while (0)

#define LOG_INFO_THROTTLE(frequency, ...)                                                                 \
    do {                                                                                                  \
        static std::chrono::steady_clock::duration throttle_duration                                      \
                = std::chrono::duration_cast<std::chrono::steady_clock::duration>(                        \
                        std::chrono::duration<double>(1.0 / frequency));                                  \
        static std::chrono::time_point<std::chrono::steady_clock> last_log_time{};                        \
        auto                                                      now = std::chrono::steady_clock::now(); \
        if (now - last_log_time >= throttle_duration) {                                                   \
            last_log_time = now;                                                                          \
            LOG_INFO(__VA_ARGS__);                                                                        \
        }                                                                                                 \
    } while (0)

#define LOG_WARN_THROTTLE(frequency, ...)                                                                 \
    do {                                                                                                  \
        static std::chrono::steady_clock::duration throttle_duration                                      \
                = std::chrono::duration_cast<std::chrono::steady_clock::duration>(                        \
                        std::chrono::duration<double>(1.0 / frequency));                                  \
        static std::chrono::time_point<std::chrono::steady_clock> last_log_time{};                        \
        auto                                                      now = std::chrono::steady_clock::now(); \
        if (now - last_log_time >= throttle_duration) {                                                   \
            last_log_time = now;                                                                          \
            LOG_WARN(__VA_ARGS__);                                                                        \
        }                                                                                                 \
    } while (0)

#define LOG_ERROR_THROTTLE(frequency, ...)                                                                \
    do {                                                                                                  \
        static std::chrono::steady_clock::duration throttle_duration                                      \
                = std::chrono::duration_cast<std::chrono::steady_clock::duration>(                        \
                        std::chrono::duration<double>(1.0 / frequency));                                  \
        static std::chrono::time_point<std::chrono::steady_clock> last_log_time{};                        \
        auto                                                      now = std::chrono::steady_clock::now(); \
        if (now - last_log_time >= throttle_duration) {                                                   \
            last_log_time = now;                                                                          \
            LOG_ERROR(__VA_ARGS__);                                                                       \
        }                                                                                                 \
    } while (0)

#define LOG_FATAL_THROTTLE(frequency, ...)                                                                \
    do {                                                                                                  \
        static std::chrono::steady_clock::duration throttle_duration                                      \
                = std::chrono::duration_cast<std::chrono::steady_clock::duration>(                        \
                        std::chrono::duration<double>(1.0 / frequency));                                  \
        static std::chrono::time_point<std::chrono::steady_clock> last_log_time{};                        \
        auto                                                      now = std::chrono::steady_clock::now(); \
        if (now - last_log_time >= throttle_duration) {                                                   \
            last_log_time = now;                                                                          \
            LOG_FATAL(__VA_ARGS__);                                                                       \
        }                                                                                                 \
    } while (0)

#define LOG_ASSERT_THROTTLE(frequency, ...)                                                               \
    do {                                                                                                  \
        static std::chrono::steady_clock::duration throttle_duration                                      \
                = std::chrono::duration_cast<std::chrono::steady_clock::duration>(                        \
                        std::chrono::duration<double>(1.0 / frequency));                                  \
        static std::chrono::time_point<std::chrono::steady_clock> last_log_time{};                        \
        auto                                                      now = std::chrono::steady_clock::now(); \
        if (now - last_log_time >= throttle_duration) {                                                   \
            last_log_time = now;                                                                          \
            LOG_ASSERT(__VA_ARGS__);                                                                      \
        }                                                                                                 \
    } while (0)

#define LOG_TRACE_ONCE(...)             \
    {                                   \
        static bool has_logged = false; \
        if (!has_logged) {              \
            LOG_TRACE(__VA_ARGS__);     \
            has_logged = true;          \
        }                               \
    }

#define LOG_DEBUG_ONCE(...)             \
    {                                   \
        static bool has_logged = false; \
        if (!has_logged) {              \
            LOG_DEBUG(__VA_ARGS__);     \
            has_logged = true;          \
        }                               \
    }

#define LOG_INFO_ONCE(...)              \
    {                                   \
        static bool has_logged = false; \
        if (!has_logged) {              \
            LOG_INFO(__VA_ARGS__);      \
            has_logged = true;          \
        }                               \
    }

#define LOG_WARN_ONCE(...)              \
    {                                   \
        static bool has_logged = false; \
        if (!has_logged) {              \
            LOG_WARN(__VA_ARGS__);      \
            has_logged = true;          \
        }                               \
    }

#define LOG_FATAL_ONCE(...)             \
    {                                   \
        static bool has_logged = false; \
        if (!has_logged) {              \
            LOG_FATAL(__VA_ARGS__);     \
            has_logged = true;          \
        }                               \
    }
#define LOG_ASSERT_ONCE(...)            \
    {                                   \
        static bool has_logged = false; \
        if (!has_logged) {              \
            LOG_ASSERT(__VA_ARGS__);    \
            has_logged = true;          \
        }                               \
    }

#define LOG_BREAK __builtin_trap();

#define LOG_TRACE(...)                                            \
    if (spdlog::get(LOG_DEFAULT_LOGGER_NAME) != nullptr) {        \
        spdlog::get(LOG_DEFAULT_LOGGER_NAME)->trace(__VA_ARGS__); \
    }
#define LOG_DEBUG(...)                                            \
    if (spdlog::get(LOG_DEFAULT_LOGGER_NAME) != nullptr) {        \
        spdlog::get(LOG_DEFAULT_LOGGER_NAME)->debug(__VA_ARGS__); \
    }
#define LOG_INFO(...)                                            \
    if (spdlog::get(LOG_DEFAULT_LOGGER_NAME) != nullptr) {       \
        spdlog::get(LOG_DEFAULT_LOGGER_NAME)->info(__VA_ARGS__); \
    }
#define LOG_WARN(...)                                            \
    if (spdlog::get(LOG_DEFAULT_LOGGER_NAME) != nullptr) {       \
        spdlog::get(LOG_DEFAULT_LOGGER_NAME)->warn(__VA_ARGS__); \
    }
#define LOG_ERROR(...)                                            \
    if (spdlog::get(LOG_DEFAULT_LOGGER_NAME) != nullptr) {        \
        spdlog::get(LOG_DEFAULT_LOGGER_NAME)->error(__VA_ARGS__); \
    }
#define LOG_FATAL(...)                                               \
    if (spdlog::get(LOG_DEFAULT_LOGGER_NAME) != nullptr) {           \
        spdlog::get(LOG_DEFAULT_LOGGER_NAME)->critical(__VA_ARGS__); \
    }
#define LOG_ASSERT(x, msg)                                                                         \
    if ((x)) {                                                                                     \
    } else {                                                                                       \
        LOG_FATAL("ASSERT - {}\n\t{}\n\tin file: {}\n\ton line: {}", #x, msg, __FILE__, __LINE__); \
        LOG_BREAK                                                                                  \
    }

// Disable logging for release builds if the macro is defined
#ifdef DISABLE_SPDLOG_ON_RELEASE_BUILD
#ifdef NDEBUG
#undef LOG_TRACE
#define LOG_TRACE(...) (void)0
#undef LOG_DEBUG
#define LOG_DEBUG(...) (void)0
#undef LOG_INFO
#define LOG_INFO(...) (void)0
#undef LOG_WARN
#define LOG_WARN(...) (void)0
#undef LOG_ERROR
#define LOG_ERROR(...) (void)0
#undef LOG_FATAL
#define LOG_FATAL(...) (void)0
#undef LOG_ASSERT
#define LOG_ASSERT(x, msg) (void)0
#endif
#endif