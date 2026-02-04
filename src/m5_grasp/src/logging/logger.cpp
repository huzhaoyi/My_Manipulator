#include "m5_grasp/logging/logger.hpp"

#include <spdlog/async.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace m5_grasp
{
namespace logging
{

namespace
{
// 信号处理器
void crashSignalHandler(int sig)
{
    // 尝试flush所有日志
    LogManager::instance().flushAll();

    // dump ring buffer
    std::string crash_file =
        "/tmp/m5_grasp_crash_" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".log";
    LogManager::instance().dumpRingBuffer(crash_file);

    // 恢复默认处理器并重新触发信号
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

// 将我们的级别转换为spdlog级别
spdlog::level::level_enum toSpdlogLevel(LogLevel level)
{
    switch (level)
    {
    case LogLevel::TRACE:
        return spdlog::level::trace;
    case LogLevel::DEBUG:
        return spdlog::level::debug;
    case LogLevel::INFO:
        return spdlog::level::info;
    case LogLevel::WARN:
        return spdlog::level::warn;
    case LogLevel::ERROR:
        return spdlog::level::err;
    case LogLevel::FATAL:
        return spdlog::level::critical;
    case LogLevel::OFF:
        return spdlog::level::off;
    default:
        return spdlog::level::info;
    }
}
} // namespace

// ========== LogManager 实现 ==========
LogManager& LogManager::instance()
{
    static LogManager mgr;
    return mgr;
}

LogManager::~LogManager()
{
    shutdown();
}

void LogManager::init(const LogConfig& config)
{
    if (initialized_.exchange(true))
    {
        return; // 已经初始化
    }

    config_ = config;

    try
    {
        // 创建日志目录
        if (config_.file_enabled)
        {
            std::filesystem::create_directories(config_.log_dir);
        }

        // 初始化异步模式
        if (config_.async_mode)
        {
            spdlog::init_thread_pool(config_.async_queue_size, 1);
        }

        // 设置sinks
        setupSinks();

        // 设置崩溃处理器
        setupCrashHandler();

        // 设置全局级别
        spdlog::set_level(toSpdlogLevel(config_.global_level));

        LOG_INFO("日志系统初始化完成: level={}, async={}, file={}, console={}",
                 levelToString(config_.global_level), config_.async_mode ? "true" : "false",
                 config_.file_enabled ? config_.log_dir : "disabled",
                 config_.console_enabled ? "true" : "false");
    }
    catch (const std::exception& e)
    {
        // 降级到控制台
        auto console = spdlog::stdout_color_mt("default");
        default_logger_ = console;
        spdlog::set_default_logger(console);
        spdlog::error("日志系统初始化失败，降级到控制台: {}", e.what());
    }
}

void LogManager::setupSinks()
{
    std::vector<spdlog::sink_ptr> sinks;

    // 1. 控制台sink（带颜色）
    if (config_.console_enabled)
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(toSpdlogLevel(config_.global_level));

        if (config_.colored_console)
        {
            console_sink->set_color_mode(spdlog::color_mode::always);

            // 自定义各级别颜色
            // TRACE: 灰色
            console_sink->set_color(spdlog::level::trace,
                                    console_sink->white); // 白色/灰色
            // DEBUG: 青色
            console_sink->set_color(spdlog::level::debug, console_sink->cyan);
            // INFO: 绿色
            console_sink->set_color(spdlog::level::info, console_sink->green);
            // WARN: 黄色加粗
            console_sink->set_color(spdlog::level::warn, console_sink->yellow_bold);
            // ERROR: 红色加粗
            console_sink->set_color(spdlog::level::err, console_sink->red_bold);
            // CRITICAL/FATAL: 红底白字
            console_sink->set_color(spdlog::level::critical, console_sink->bold_on_red);
        }
        sinks.push_back(console_sink);
    }

    // 2. 主文件sink（滚动，记录所有级别）
    if (config_.file_enabled)
    {
        std::string log_file = config_.log_dir + "/" + config_.file_prefix + ".log";

        if (config_.daily_rotation)
        {
            auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
                log_file, 0, 0, false, static_cast<uint16_t>(config_.max_files));
            file_sink->set_level(toSpdlogLevel(config_.global_level));
            sinks.push_back(file_sink);
        }
        else
        {
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                log_file, config_.max_file_size_mb * 1024 * 1024, config_.max_files);
            file_sink->set_level(toSpdlogLevel(config_.global_level));
            sinks.push_back(file_sink);
        }

        // 3. 警告文件sink（只记录WARN及以上）
        std::string warn_file = config_.log_dir + "/" + config_.file_prefix + "_warn.log";
        auto warn_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            warn_file, config_.max_file_size_mb * 1024 * 1024, config_.max_files);
        warn_sink->set_level(spdlog::level::warn); // 只记录WARN及以上
        sinks.push_back(warn_sink);

        // 4. 错误文件sink（只记录ERROR及以上）
        std::string error_file = config_.log_dir + "/" + config_.file_prefix + "_error.log";
        auto error_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            error_file, config_.max_file_size_mb * 1024 * 1024, config_.max_files);
        error_sink->set_level(spdlog::level::err); // 只记录ERROR及以上
        sinks.push_back(error_sink);
    }

    // 3. Ring buffer sink（用于崩溃dump）
    if (config_.ring_buffer_enabled)
    {
        auto ring_sink =
            std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(config_.ring_buffer_size);
        ring_sink->set_level(spdlog::level::debug); // 记录更多级别
        ring_buffer_logger_ = std::make_shared<spdlog::logger>("ringbuffer", ring_sink);
    }

    // 创建默认logger
    if (config_.async_mode && !sinks.empty())
    {
        default_logger_ = std::make_shared<spdlog::async_logger>(
            "default", sinks.begin(), sinks.end(), spdlog::thread_pool(),
            config_.drop_on_full ? spdlog::async_overflow_policy::overrun_oldest
                                 : spdlog::async_overflow_policy::block);
    }
    else if (!sinks.empty())
    {
        default_logger_ = std::make_shared<spdlog::logger>("default", sinks.begin(), sinks.end());
    }
    else
    {
        // 至少有控制台
        default_logger_ = spdlog::stdout_color_mt("default");
    }

    // 设置格式
    // 格式: [时间] [级别] [模块] [线程] [源文件:行] 消息
    std::string pattern;
    if (config_.include_source_location && config_.include_thread_id)
    {
        pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] [tid=%t] [%s:%#] %v";
    }
    else if (config_.include_thread_id)
    {
        pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] [tid=%t] %v";
    }
    else
    {
        pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v";
    }
    default_logger_->set_pattern(pattern);
    default_logger_->set_level(toSpdlogLevel(config_.global_level));

    // 注册为默认logger
    spdlog::set_default_logger(default_logger_);

    // 存储到map
    std::lock_guard<std::mutex> lock(loggers_mutex_);
    loggers_["default"] = default_logger_;
}

void LogManager::setupCrashHandler()
{
    // 注册信号处理器
    std::signal(SIGSEGV, crashSignalHandler);
    std::signal(SIGABRT, crashSignalHandler);
    std::signal(SIGFPE, crashSignalHandler);
    std::signal(SIGILL, crashSignalHandler);

    // 注册std::terminate处理器
    std::set_terminate(
        []()
        {
            LogManager::instance().flushAll();
            LogManager::instance().dumpRingBuffer("/tmp/m5_grasp_terminate.log");
            std::abort();
        });
}

void LogManager::shutdown()
{
    if (!initialized_.exchange(false))
    {
        return;
    }

    LOG_INFO("日志系统关闭");
    flushAll();

    std::lock_guard<std::mutex> lock(loggers_mutex_);
    loggers_.clear();
    default_logger_.reset();
    ring_buffer_logger_.reset();

    spdlog::shutdown();
}

std::shared_ptr<spdlog::logger> LogManager::getLogger(const std::string& module_name)
{
    if (!initialized_)
    {
        return nullptr;
    }

    // 快速路径：默认logger
    if (module_name == "default" || module_name.empty())
    {
        return default_logger_;
    }

    // 查找已存在的logger
    {
        std::lock_guard<std::mutex> lock(loggers_mutex_);
        auto it = loggers_.find(module_name);
        if (it != loggers_.end())
        {
            return it->second;
        }
    }

    // 创建新的模块logger（克隆默认logger的sinks）
    std::lock_guard<std::mutex> lock(loggers_mutex_);

    // 双重检查
    auto it = loggers_.find(module_name);
    if (it != loggers_.end())
    {
        return it->second;
    }

    // 创建新logger
    auto new_logger = default_logger_->clone(module_name);
    loggers_[module_name] = new_logger;
    spdlog::register_logger(new_logger);

    return new_logger;
}

void LogManager::setGlobalLevel(LogLevel level)
{
    config_.global_level = level;
    spdlog::set_level(toSpdlogLevel(level));
    LOG_INFO("全局日志级别已设置为: {}", levelToString(level));
}

void LogManager::setModuleLevel(const std::string& module_name, LogLevel level)
{
    auto logger = getLogger(module_name);
    if (logger)
    {
        logger->set_level(toSpdlogLevel(level));
        LOG_INFO("模块 {} 日志级别已设置为: {}", module_name, levelToString(level));
    }
}

void LogManager::flushAll()
{
    spdlog::apply_all([](std::shared_ptr<spdlog::logger> l) { l->flush(); });
}

void LogManager::dumpRingBuffer(const std::string& filename)
{
    if (!ring_buffer_logger_)
    {
        return;
    }

    try
    {
        // 获取ring buffer sink
        auto sinks = ring_buffer_logger_->sinks();
        if (sinks.empty())
            return;

        auto ring_sink = std::dynamic_pointer_cast<spdlog::sinks::ringbuffer_sink_mt>(sinks[0]);
        if (!ring_sink)
            return;

        // 获取最近的日志
        auto messages = ring_sink->last_formatted();

        // 写入文件
        std::ofstream out(filename);
        if (out.is_open())
        {
            out << "=== M5 Grasp Crash Dump ===\n";
            out << "Timestamp: " << std::chrono::system_clock::now().time_since_epoch().count()
                << "\n";
            out << "Task ID: " << current_task_id_ << "\n";
            out << "=== Recent Logs ===\n";
            for (const auto& msg : messages)
            {
                out << msg;
            }
            out.close();
        }
    }
    catch (...)
    {
        // 崩溃处理中不能再抛异常
    }
}

void LogManager::setTaskId(const std::string& task_id)
{
    current_task_id_ = task_id;
    LOG_INFO("任务追踪开始: task_id={}", task_id);
}

void LogManager::setROS2Node(void* node)
{
    ros2_node_ = node;
}

// ========== ThrottleController 实现 ==========
std::string ThrottleController::makeKey(const char* file, int line)
{
    return std::string(file) + ":" + std::to_string(line);
}

bool ThrottleController::shouldLog(const char* file, int line, int interval_ms)
{
    std::string key = makeKey(file, line);
    auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = states_[key];

    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - state.last_log_time).count();

    if (elapsed >= interval_ms)
    {
        state.last_log_time = now;
        return true;
    }
    return false;
}

bool ThrottleController::shouldLogOnce(const char* file, int line)
{
    std::string key = makeKey(file, line);

    std::lock_guard<std::mutex> lock(mutex_);
    auto& state = states_[key];

    if (!state.logged_once)
    {
        state.logged_once = true;
        return true;
    }
    return false;
}

// ========== 辅助函数 ==========
LogLevel stringToLevel(const std::string& level_str)
{
    std::string lower = level_str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "trace")
        return LogLevel::TRACE;
    if (lower == "debug")
        return LogLevel::DEBUG;
    if (lower == "info")
        return LogLevel::INFO;
    if (lower == "warn" || lower == "warning")
        return LogLevel::WARN;
    if (lower == "error" || lower == "err")
        return LogLevel::ERROR;
    if (lower == "fatal" || lower == "critical")
        return LogLevel::FATAL;
    if (lower == "off")
        return LogLevel::OFF;

    return LogLevel::INFO; // 默认
}

std::string levelToString(LogLevel level)
{
    switch (level)
    {
    case LogLevel::TRACE:
        return "TRACE";
    case LogLevel::DEBUG:
        return "DEBUG";
    case LogLevel::INFO:
        return "INFO";
    case LogLevel::WARN:
        return "WARN";
    case LogLevel::ERROR:
        return "ERROR";
    case LogLevel::FATAL:
        return "FATAL";
    case LogLevel::OFF:
        return "OFF";
    default:
        return "UNKNOWN";
    }
}

} // namespace logging
} // namespace m5_grasp
