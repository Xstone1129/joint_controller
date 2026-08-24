#ifndef ROBOT_CONTROL_MSG__ROBOT_COMMAND_GUARD_HPP_
#define ROBOT_CONTROL_MSG__ROBOT_COMMAND_GUARD_HPP_

#include <cerrno>
#include <cstring>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace robot_control_msg
{
namespace safety
{

class RobotCommandGuard
{
public:
  static constexpr const char * kDefaultLockPath =
    "/tmp/joint_controller_robot_command.lock";

  explicit RobotCommandGuard(
    std::string owner, const std::string & lock_path = kDefaultLockPath)
  : owner_(std::move(owner))
  {
    file_descriptor_ = open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
    if (file_descriptor_ < 0) {
      error_ = "cannot open robot command lock '" + lock_path + "': " +
        std::strerror(errno);
      return;
    }

    if (flock(file_descriptor_, LOCK_EX | LOCK_NB) != 0) {
      if (errno == EWOULDBLOCK || errno == EAGAIN) {
        const std::string current_owner = readOwner();
        error_ = "another robot command is already in progress";
        if (!current_owner.empty()) {
          error_ += ": " + current_owner;
        }
      } else {
        error_ = "cannot acquire robot command lock: " + std::string(std::strerror(errno));
      }
      close(file_descriptor_);
      file_descriptor_ = -1;
      return;
    }

    acquired_ = true;
    if (ftruncate(file_descriptor_, 0) == 0 && lseek(file_descriptor_, 0, SEEK_SET) >= 0) {
      const std::string label = owner_ + "\n";
      const ssize_t unused = write(file_descriptor_, label.data(), label.size());
      (void)unused;
    }
  }

  RobotCommandGuard(const RobotCommandGuard &) = delete;
  RobotCommandGuard & operator=(const RobotCommandGuard &) = delete;
  RobotCommandGuard(RobotCommandGuard &&) = delete;
  RobotCommandGuard & operator=(RobotCommandGuard &&) = delete;

  ~RobotCommandGuard()
  {
    if (file_descriptor_ < 0) {
      return;
    }
    if (acquired_) {
      const int truncate_result = ftruncate(file_descriptor_, 0);
      (void)truncate_result;
      flock(file_descriptor_, LOCK_UN);
    }
    close(file_descriptor_);
  }

  bool acquired() const noexcept
  {
    return acquired_;
  }

  const std::string & error() const noexcept
  {
    return error_;
  }

private:
  std::string readOwner()
  {
    if (lseek(file_descriptor_, 0, SEEK_SET) < 0) {
      return {};
    }

    char buffer[256] = {};
    const ssize_t count = read(file_descriptor_, buffer, sizeof(buffer) - 1);
    if (count <= 0) {
      return {};
    }

    std::string result(buffer, static_cast<std::size_t>(count));
    while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
      result.pop_back();
    }
    return result;
  }

  std::string owner_;
  std::string error_;
  int file_descriptor_{-1};
  bool acquired_{false};
};

}  // namespace safety
}  // namespace robot_control_msg

#endif  // ROBOT_CONTROL_MSG__ROBOT_COMMAND_GUARD_HPP_
