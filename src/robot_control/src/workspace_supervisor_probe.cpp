// Process and ROS graph probing for the workspace supervisor.
//
// These helpers answer "is anything still running that belongs to a workspace
// stack?" from two independent sources: the /proc command lines of the
// processes this node manages, and the ROS graph's service servers (clients are
// deliberately ignored so the always-on gateway cannot look like an active
// control stack).

#include "robot_control/workspace_supervisor_ubuntu.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace robot_control
{
using namespace workspace_supervisor;

std::string WorkspaceSupervisorUbuntu::join(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  return stream.str();
}

std::string WorkspaceSupervisorUbuntu::read_process_command(pid_t pid)
{
  std::ifstream stream("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
  std::string command(
    (std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  std::replace(command.begin(), command.end(), '\0', ' ');
  while (!command.empty() && command.back() == ' ') {
    command.pop_back();
  }
  return command;
}

std::vector<ManagedProcess> WorkspaceSupervisorUbuntu::collect_processes_matching(
  const std::vector<std::string> & fragments)
{
  std::vector<ManagedProcess> matches;
  DIR * proc = opendir("/proc");
  if (proc == nullptr) {
    return matches;
  }

  while (const auto * entry = readdir(proc)) {
    if (entry->d_name[0] < '0' || entry->d_name[0] > '9') {
      continue;
    }
    char * end = nullptr;
    const long parsed_pid = std::strtol(entry->d_name, &end, 10);
    if (end == nullptr || *end != '\0' || parsed_pid <= 1) {
      continue;
    }
    const auto command = read_process_command(static_cast<pid_t>(parsed_pid));
    if (command.empty()) {
      continue;
    }
    const bool matched = std::any_of(
      fragments.begin(), fragments.end(), [&command](const std::string & fragment) {
        return command.find(fragment) != std::string::npos;
      });
    if (matched) {
      matches.push_back({static_cast<pid_t>(parsed_pid), command});
    }
  }
  closedir(proc);
  std::sort(
    matches.begin(), matches.end(),
    [](const ManagedProcess & left, const ManagedProcess & right) {
      return left.pid < right.pid;
    });
  return matches;
}

std::vector<ManagedProcess> WorkspaceSupervisorUbuntu::collect_managed_stack_processes()
{
  return collect_processes_matching({
    "/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver",
    "ros2 launch erobot_controller load_controller_arm.launch.py",
    "ros2 launch robot_control robot_control.launch.py",
    "ros2 launch joint_hardware lift_ethercat.launch.py",
    "/controller_manager/ros2_control_node",
    "/controller_manager/spawner",
    "/lift/controller_manager/ros2_control_node",
    "/robot_state_publisher/robot_state_publisher",
    "/robot_control/cartesian_single_control_srv",
    "/robot_control/cartesian_path_absolute_control_srv",
    "/robot_control/cartesian_path_increment_control_srv",
    "/robot_control/joint_absolute_control_srv",
    "/robot_control/joint_batch_control_srv",
    "/robot_control/cartesian_moveL_path",
    "/robot_control_msg/arm_control_mode_service",
  });
}

std::vector<ManagedProcess> WorkspaceSupervisorUbuntu::collect_igh_driver_processes()
{
  return collect_processes_matching({
    "/home/user/joint_controller/src/erobot_igh_driver/build/igh_driver"});
}

std::string WorkspaceSupervisorUbuntu::describe_process(const ManagedProcess & process)
{
  return "residual process pid=" + std::to_string(process.pid) +
         " cmd='" + process.command + "'";
}

ServiceServerGraph WorkspaceSupervisorUbuntu::collect_service_server_graph()
{
  ServiceServerGraph graph;
  const auto nodes = get_node_graph_interface()->get_node_names_and_namespaces();
  for (const auto & node : nodes) {
    const auto services = get_service_names_and_types_by_node(node.first, node.second);
    const auto owner = fully_qualified_node_name(node.first, node.second);
    for (const auto & service : services) {
      auto & types = graph.types[service.first];
      for (const auto & type : service.second) {
        if (std::find(types.begin(), types.end(), type) == types.end()) {
          types.push_back(type);
        }
      }

      auto & owners = graph.owners[service.first];
      if (std::find(owners.begin(), owners.end(), owner) == owners.end()) {
        owners.push_back(owner);
      }
    }
  }
  return graph;
}

}  // namespace robot_control
