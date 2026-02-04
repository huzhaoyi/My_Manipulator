#include "m5_grasp/fsm/target_tracker.hpp"
#include "m5_grasp/logging/logger.hpp"

namespace m5_grasp
{

TargetTracker::TargetTracker() : config_(), latest_stamp_(0, 0, RCL_ROS_TIME)
{
}

TargetTracker::TargetTracker(const Config& config)
    : config_(config), latest_stamp_(0, 0, RCL_ROS_TIME)
{
}

void TargetTracker::setConfig(const Config& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

void TargetTracker::push(const m5_msgs::msg::CablePoseWithYaw& msg, const rclcpp::Time& stamp)
{
    std::lock_guard<std::mutex> lock(mutex_);

    latest_target_ = msg;
    latest_stamp_ = stamp;
    has_target_ = true;

    // 推入滑窗
    window_.push_back(msg);
    while (window_.size() > config_.window_size)
    {
        window_.pop_front();
    }
}

bool TargetTracker::hasStableTarget(const rclcpp::Time& now) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 条件1：滑窗内至少有足够帧数
    if (window_.size() < config_.window_size)
    {
        LOG_NAMED_DEBUG("target_tracker", "目标不稳定: 滑窗帧数不足 ({}/{})", window_.size(),
                        config_.window_size);
        return false;
    }

    // 条件2：目标时间戳新鲜度
    double age = (now - latest_stamp_).seconds();
    if (age > config_.stale_timeout)
    {
        LOG_NAMED_DEBUG("target_tracker", "目标不稳定: 目标过期 ({:.2f}s > {:.2f}s)", age,
                        config_.stale_timeout);
        return false;
    }

    // 条件3&4：计算滑窗内最大偏移
    const auto& base = window_.front();
    double max_d = 0.0;
    double max_yaw = 0.0;

    for (const auto& t : window_)
    {
        double dx = t.position.x - base.position.x;
        double dy = t.position.y - base.position.y;
        double dz = t.position.z - base.position.z;
        double d = std::sqrt(dx * dx + dy * dy + dz * dz);
        max_d = std::max(max_d, d);

        double dyaw = std::abs(normalize_angle(t.yaw - base.yaw));
        max_yaw = std::max(max_yaw, dyaw);
    }

    // 阈值判断
    if (max_d >= config_.stable_pos_threshold || max_yaw >= config_.stable_yaw_threshold)
    {
        LOG_NAMED_DEBUG("target_tracker",
                        "目标不稳定: max_d={:.3f}m (阈值{:.3f}), max_yaw={:.1f}° (阈值{:.1f}°)",
                        max_d, config_.stable_pos_threshold, max_yaw * 180.0 / M_PI,
                        config_.stable_yaw_threshold * 180.0 / M_PI);
        return false;
    }

    LOG_NAMED_INFO("target_tracker", "目标稳定: max_d={:.3f}m, max_yaw={:.1f}°", max_d,
                   max_yaw * 180.0 / M_PI);
    return true;
}

std::optional<m5_msgs::msg::CablePoseWithYaw> TargetTracker::getStableTarget() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_target_ && window_.size() >= config_.window_size)
    {
        return latest_target_;
    }
    return std::nullopt;
}

std::optional<m5_msgs::msg::CablePoseWithYaw> TargetTracker::getLatestTarget() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_target_)
    {
        return latest_target_;
    }
    return std::nullopt;
}

rclcpp::Time TargetTracker::getLatestStamp() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_stamp_;
}

bool TargetTracker::isStale(const rclcpp::Time& now) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_target_)
    {
        return true;
    }
    double age = (now - latest_stamp_).seconds();
    return age > config_.stale_timeout;
}

bool TargetTracker::isJumping(const m5_msgs::msg::CablePoseWithYaw& ref) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_target_)
    {
        return true; // 没有目标视为跳变
    }

    // 计算位置偏移
    double dx = latest_target_.position.x - ref.position.x;
    double dy = latest_target_.position.y - ref.position.y;
    double dz = latest_target_.position.z - ref.position.z;
    double pos_jump = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 计算yaw偏移
    double yaw_jump = std::abs(normalize_angle(latest_target_.yaw - ref.yaw));

    if (pos_jump >= config_.jump_pos_threshold || yaw_jump >= config_.jump_yaw_threshold)
    {
        LOG_NAMED_WARN("target_tracker",
                       "目标跳变: pos={:.3f}m (阈值{:.3f}), yaw={:.1f}° (阈值{:.1f}°)", pos_jump,
                       config_.jump_pos_threshold, yaw_jump * 180.0 / M_PI,
                       config_.jump_yaw_threshold * 180.0 / M_PI);
        return true;
    }

    return false;
}

bool TargetTracker::is_valid(const rclcpp::Time& now,
                            const m5_msgs::msg::CablePoseWithYaw& ref) const
{
    if (isStale(now))
    {
        LOG_NAMED_WARN("target_tracker", "目标无效: 已过期");
        return false;
    }
    if (isJumping(ref))
    {
        LOG_NAMED_WARN("target_tracker", "目标无效: 已跳变");
        return false;
    }
    return true;
}

void TargetTracker::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    window_.clear();
    has_target_ = false;
}

size_t TargetTracker::windowSize() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return window_.size();
}

double TargetTracker::normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

} // namespace m5_grasp
