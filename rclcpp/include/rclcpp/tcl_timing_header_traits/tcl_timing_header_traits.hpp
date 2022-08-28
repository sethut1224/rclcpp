#ifndef RCLCPP__TCL_TIMING_HEADER_TRAITS__HPP_
#define RCLCPP__TCL_TIMING_HEADER_TRATIS__HPP_

#include <type_traits>
#include "tcl_msgs/msg/profile.hpp"
#include "tcl_msgs/msg/timing_coordination_header.hpp"
#include "rcpputils/pointer_traits.hpp"

using tcl_msgs::msg::Profile;
using tcl_msgs::msg::TimingCoordinationHeader;

template<typename M, typename = void>
struct isProfileMessage : public std::false_type{};

template<typename M>
struct isProfileMessage<M, typename std::enable_if<std::is_same<M, Profile>::value>::type> : std::true_type {};

template<typename M, typename = void>
struct isTimingHeader : public std::false_type{};

template<typename M>
struct isTimingHeader<M, typename std::enable_if<std::is_same<M, TimingCoordinationHeader>::value>::type> : std::true_type {};

#endif