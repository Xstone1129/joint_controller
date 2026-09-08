#ifndef ROBOT_LOWER_GATEWAY__NATIVE_SERVICE_FORWARDER_HPP_
#define ROBOT_LOWER_GATEWAY__NATIVE_SERVICE_FORWARDER_HPP_

#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/client.hpp"

namespace robot_lower_gateway
{

template<typename ServiceT>
std::shared_ptr<typename ServiceT::Response> callNativeServiceBounded(
  const typename ServiceT::Request::SharedPtr & request,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const std::string & native_service,
  std::chrono::milliseconds timeout,
  std::string & error)
{
  if (!client) {
    error = "native client is not configured for " + native_service;
    return nullptr;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const auto remaining = [&deadline]() {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return std::chrono::milliseconds(0);
      }
      return std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    };

  try {
    if (!client->wait_for_service(remaining())) {
      error = "native service unavailable: " + native_service;
      return nullptr;
    }

    auto future = client->async_send_request(request);
    if (future.wait_for(remaining()) != std::future_status::ready) {
      client->remove_pending_request(future);
      error = "timeout waiting for native service " + native_service;
      return nullptr;
    }
    auto response = future.get();
    if (!response) {
      error = "native service returned an empty response: " + native_service;
      return nullptr;
    }
    return response;
  } catch (const std::exception & exception) {
    error = "native service call failed for " + native_service + ": " + exception.what();
    return nullptr;
  }
}

}  // namespace robot_lower_gateway

#endif  // ROBOT_LOWER_GATEWAY__NATIVE_SERVICE_FORWARDER_HPP_
