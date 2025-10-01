#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>


namespace util
{

template<typename T>
using RclPubPtr = typename rclcpp::Publisher<T>::SharedPtr;
template<typename T>
using RclSubPtr = typename rclcpp::Subscription<T>::SharedPtr;
using RclTimerPtr = rclcpp::TimerBase::SharedPtr;

template<typename T>
inline void declare_param(
    rclcpp::Node* node,
    const std::string param_name,
    T& param,
    const T& default_value)
{
    node->declare_parameter(param_name, default_value);
    node->get_parameter(param_name, param);
}

template<typename T>
inline T declare_and_get_param(
    rclcpp::Node* node,
    const std::string param_name,
    const T& default_value)
{
    T param;
    node->declare_parameter(param_name, default_value);
    node->get_parameter(param_name, param);
    return param;
}

};  // namespace util
