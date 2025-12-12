#ifndef GETPARM_UTILS_HPP_
#define GETPARM_UTILS_HPP_
#include <rclcpp/rclcpp.hpp>
#include <string>
namespace apex_utils
{
    template<typename T>
    void get_param(rclcpp::Node::SharedPtr node,const std::string& ParamName,T& variable,const T& default_value)
    {
        try
        {
            if (!node->has_parameter(ParamName))
            {
                variable = node->declare_parameter<T>(ParamName,default_value);
            }else
            {
                node->get_parameter(ParamName,variable);

            }
        }catch (const rclcpp::exceptions::InvalidParameterTypeException&e)
        {
            RCLCPP_ERROR(node->get_logger(),"Parameter type mismatch for %s:%s",ParamName.c_str(),e.what());
            throw e;
        }
    }
}
#endif
