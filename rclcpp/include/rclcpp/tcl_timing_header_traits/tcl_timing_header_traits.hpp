#ifndef RCLCPP__TCL_TIMING_HEADER_TRAITS__HPP_
#define RCLCPP__TCL_TIMING_HEADER_TRATIS__HPP_

#include <type_traits>
#include "tcl_msgs/msg/timing_coordination_header.hpp"
#include "std_msgs/msg/header.hpp"
#include "rcpputils/pointer_traits.hpp"

using tcl_msgs::msg::TimingCoordinationHeader;

template<typename M, typename = void> 
struct HasHeader : public std::false_type {};

template<typename M>
struct HasHeader<M, decltype((void) M::header)>: std::true_type {};

template<typename M, typename Enable = void>
struct TimingHeader
{
    static tcl_msgs::msg::TimingCoordinationHeader value(const M &m) {
        (void)m;
        return TimingCoordinationHeader();
    }
};

template<typename M>
struct TimingHeader<M, typename std::enable_if<HasHeader<M>::value>::type >
{
  static tcl_msgs::msg::TimingCoordinationHeader value(const M& m) {
    return m.header.timing_header;
  }
};


template<typename M, typename Enable = void>
struct Propagate
{
  static void  propagate(M& m, const TimingCoordinationHeader& u) {(void)m; (void)u; return; }
};

template<typename M>
struct Propagate<M, typename std::enable_if<HasHeader<M>::value>::type >
{
  static void propagate(M& m, const TimingCoordinationHeader& u) { 
      m.header.timing_header = u;
    }
};
     
#endif