#pragma once

namespace m5_grasp {

/**
 * @brief 夹爪抽象接口
 * 
 * FSM 只依赖此接口，不关心具体实现（MoveIt/Service/Serial）
 */
class IGripper {
public:
  virtual ~IGripper() = default;

  /**
   * @brief 打开夹爪
   * @return true 如果命令发送成功
   */
  virtual bool open() = 0;

  /**
   * @brief 闭合夹爪
   * @return true 如果命令发送成功
   */
  virtual bool close() = 0;

  /**
   * @brief 检查夹爪是否打开
   * @return true 如果夹爪处于打开状态
   */
  virtual bool isOpen() const { return false; }

  /**
   * @brief 检查夹爪是否闭合
   * @return true 如果夹爪处于闭合状态
   */
  virtual bool isClosed() const { return false; }

  /**
   * @brief 检查夹爪是否已到达“打开”目标（用于 FSM 非阻塞到位判断，仅依据稳定性验证标志）
   * @return true 若打开动作已通过稳定性验证并尚未被 FSM 消费
   */
  virtual bool isOpenToTarget() { return false; }

  /**
   * @brief 检查夹爪是否已到达“闭合”目标（用于 FSM 非阻塞到位判断，仅依据稳定性验证标志）
   * @return true 若闭合动作已通过稳定性验证并尚未被 FSM 消费
   */
  virtual bool isClosedToTarget() { return false; }

  /**
   * @brief 等待夹爪动作完成
   * @param timeout_ms 超时时间（毫秒）
   * @return true 如果在超时内完成
   */
  virtual bool waitForCompletion(int timeout_ms = 500) { 
    // 默认实现：简单等待
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    return true;
  }
};

}  // namespace m5_grasp
