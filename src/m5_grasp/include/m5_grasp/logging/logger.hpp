#pragma once

/**
 * @file logger.hpp
 * @brief 统一日志API层 - 机械臂控制项目日志门面
 *
 * 设计原则：
 * 1. 控制线程永不阻塞 - 使用异步日志
 * 2. 可追溯 - 时间戳、线程ID、模块名、源文件行号
 * 3. 可分级 - DEBUG/INFO/WARN/ERROR，支持动态调整
 * 4. ROS2兼容 - 可选桥接到ROS2 logger
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

// 前向声明spdlog
namespace spdlog
{
class logger;
}

namespace m5_grasp
{
namespace logging
{

// ========== 日志级别 ==========
enum class LogLevel
{
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    FATAL = 5,
    OFF = 6
};

// ========== 日志配置 ==========
struct LogConfig
{
    // 全局配置
    LogLevel global_level = LogLevel::INFO;
    bool async_mode = true;
    size_t async_queue_size = 8192;
    bool drop_on_full = true; // 队列满时丢弃低等级日志

    // 文件配置
    bool file_enabled = true;
    std::string log_dir = "/tmp/m5_grasp_logs";
    std::string file_prefix = "m5_grasp";
    size_t max_file_size_mb = 50;
    size_t max_files = 20;
    bool daily_rotation = false;

    // 控制台配置
    bool console_enabled = true;
    bool colored_console = true;

    // ROS2桥接配置
    bool ros2_bridge_enabled = false;
    LogLevel ros2_bridge_level = LogLevel::WARN; // 只转发WARN及以上

    // 格式配置
    bool include_source_location = true; // DEBUG时开启
    bool include_thread_id = true;

    // Ring buffer配置（用于崩溃时dump）
    bool ring_buffer_enabled = true;
    size_t ring_buffer_size = 1000; // 最近N条日志
};

// ========== 日志管理器（单例） ==========
class LogManager
{
  public:
    static LogManager& instance();

    // 初始化（在程序启动时调用一次）
    void init(const LogConfig& config = LogConfig{});
    void shutdown();

    // 获取模块logger
    std::shared_ptr<spdlog::logger> getLogger(const std::string& module_name = "default");

    // 动态调整级别
    void setGlobalLevel(LogLevel level);
    void setModuleLevel(const std::string& module_name, LogLevel level);
    LogLevel getGlobalLevel() const
    {
        return config_.global_level;
    }

    // 崩溃时调用
    void flushAll();
    void dumpRingBuffer(const std::string& filename);

    // 任务追踪
    void setTaskId(const std::string& task_id);
    std::string getTaskId() const
    {
        return current_task_id_;
    }
    void clearTaskId()
    {
        current_task_id_.clear();
    }

    // ROS2节点设置（用于ROS2桥接）
    void setROS2Node(void* node);

    // 配置获取
    const LogConfig& getConfig() const
    {
        return config_;
    }
    bool isInitialized() const
    {
        return initialized_;
    }

  private:
    LogManager() = default;
    ~LogManager();
    LogManager(const LogManager&) = delete;
    LogManager& operator=(const LogManager&) = delete;

    void setupSinks();
    void setupCrashHandler();

    LogConfig config_;
    std::atomic<bool> initialized_{false};
    std::string current_task_id_;
    void* ros2_node_ = nullptr;

    std::mutex loggers_mutex_;
    std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> loggers_;

    std::shared_ptr<spdlog::logger> default_logger_;
    std::shared_ptr<spdlog::logger> ring_buffer_logger_;
};

// ========== 限频控制 ==========
class ThrottleController
{
  public:
    bool shouldLog(const char* file, int line, int interval_ms);
    bool shouldLogOnce(const char* file, int line);

    static ThrottleController& instance()
    {
        static ThrottleController ctrl;
        return ctrl;
    }

  private:
    struct ThrottleState
    {
        std::chrono::steady_clock::time_point last_log_time;
        bool logged_once = false;
    };

    std::mutex mutex_;
    std::unordered_map<std::string, ThrottleState> states_;

    std::string makeKey(const char* file, int line);
};

// ========== 辅助函数 ==========
LogLevel stringToLevel(const std::string& level_str);
std::string levelToString(LogLevel level);

} // namespace logging
} // namespace m5_grasp

// ========== 统一日志宏 ==========
// 需要include spdlog头文件
#include <spdlog/spdlog.h>

// 基础宏 - 使用spdlog::level::level_enum
#define M5_LOG_IMPL(spdlog_level, module, ...)                                                     \
    do                                                                                             \
    {                                                                                              \
        auto& mgr = m5_grasp::logging::LogManager::instance();                                     \
        if (mgr.isInitialized())                                                                   \
        {                                                                                          \
            auto logger = mgr.getLogger(module);                                                   \
            if (logger)                                                                            \
            {                                                                                      \
                logger->log(spdlog_level, __VA_ARGS__);                                            \
            }                                                                                      \
        }                                                                                          \
    } while (0)

// 简化宏（使用默认模块）
#define LOG_TRACE(...) M5_LOG_IMPL(spdlog::level::trace, "default", __VA_ARGS__)
#define LOG_DEBUG(...) M5_LOG_IMPL(spdlog::level::debug, "default", __VA_ARGS__)
#define LOG_INFO(...) M5_LOG_IMPL(spdlog::level::info, "default", __VA_ARGS__)
#define LOG_WARN(...) M5_LOG_IMPL(spdlog::level::warn, "default", __VA_ARGS__)
#define LOG_ERROR(...) M5_LOG_IMPL(spdlog::level::err, "default", __VA_ARGS__)
#define LOG_FATAL(...) M5_LOG_IMPL(spdlog::level::critical, "default", __VA_ARGS__)

// 带模块名的宏
#define LOG_NAMED_TRACE(module, ...) M5_LOG_IMPL(spdlog::level::trace, module, __VA_ARGS__)
#define LOG_NAMED_DEBUG(module, ...) M5_LOG_IMPL(spdlog::level::debug, module, __VA_ARGS__)
#define LOG_NAMED_INFO(module, ...) M5_LOG_IMPL(spdlog::level::info, module, __VA_ARGS__)
#define LOG_NAMED_WARN(module, ...) M5_LOG_IMPL(spdlog::level::warn, module, __VA_ARGS__)
#define LOG_NAMED_ERROR(module, ...) M5_LOG_IMPL(spdlog::level::err, module, __VA_ARGS__)
#define LOG_NAMED_FATAL(module, ...) M5_LOG_IMPL(spdlog::level::critical, module, __VA_ARGS__)

// 限频宏（每interval_ms毫秒最多打一次）
#define LOG_THROTTLE_IMPL(spdlog_level, module, interval_ms, ...)                                  \
    do                                                                                             \
    {                                                                                              \
        if (m5_grasp::logging::ThrottleController::instance().shouldLog(__FILE__, __LINE__,        \
                                                                        interval_ms))              \
        {                                                                                          \
            M5_LOG_IMPL(spdlog_level, module, __VA_ARGS__);                                        \
        }                                                                                          \
    } while (0)

#define LOG_THROTTLE_INFO(interval_ms, ...)                                                        \
    LOG_THROTTLE_IMPL(spdlog::level::info, "default", interval_ms, __VA_ARGS__)
#define LOG_THROTTLE_WARN(interval_ms, ...)                                                        \
    LOG_THROTTLE_IMPL(spdlog::level::warn, "default", interval_ms, __VA_ARGS__)
#define LOG_THROTTLE_DEBUG(interval_ms, ...)                                                       \
    LOG_THROTTLE_IMPL(spdlog::level::debug, "default", interval_ms, __VA_ARGS__)

// 只打一次的宏
#define LOG_ONCE_IMPL(spdlog_level, module, ...)                                                   \
    do                                                                                             \
    {                                                                                              \
        if (m5_grasp::logging::ThrottleController::instance().shouldLogOnce(__FILE__, __LINE__))   \
        {                                                                                          \
            M5_LOG_IMPL(spdlog_level, module, __VA_ARGS__);                                        \
        }                                                                                          \
    } while (0)

#define LOG_ONCE_INFO(...) LOG_ONCE_IMPL(spdlog::level::info, "default", __VA_ARGS__)
#define LOG_ONCE_WARN(...) LOG_ONCE_IMPL(spdlog::level::warn, "default", __VA_ARGS__)
#define LOG_ONCE_ERROR(...) LOG_ONCE_IMPL(spdlog::level::err, "default", __VA_ARGS__)

// 条件日志
#define LOG_IF(cond, spdlog_level, ...)                                                            \
    do                                                                                             \
    {                                                                                              \
        if (cond)                                                                                  \
            M5_LOG_IMPL(spdlog_level, "default", __VA_ARGS__);                                     \
    } while (0)

// 带任务ID的结构化日志
#define LOG_TASK_INFO(key_values, ...)                                                             \
    do                                                                                             \
    {                                                                                              \
        auto& mgr = m5_grasp::logging::LogManager::instance();                                     \
        std::string task_id = mgr.getTaskId();                                                     \
        if (!task_id.empty())                                                                      \
        {                                                                                          \
            LOG_INFO("[task={}] " key_values, task_id, __VA_ARGS__);                               \
        }                                                                                          \
        else                                                                                       \
        {                                                                                          \
            LOG_INFO(key_values, __VA_ARGS__);                                                     \
        }                                                                                          \
    } while (0)
