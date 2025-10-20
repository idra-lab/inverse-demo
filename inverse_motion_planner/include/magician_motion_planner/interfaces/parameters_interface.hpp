#ifndef MAGICIAN_MOTION_PLANNER_PARAMETER_INTERFACE_HPP
#define MAGICIAN_MOTION_PLANNER_PARAMETER_INTERFACE_HPP

#include <string>

#include <mdv/macros.hpp>

class PlannerParameterInterface {
public:
    virtual ~PlannerParameterInterface() = default;

    MDV_NODISCARD virtual std::string get_base_link() const = 0;
    MDV_NODISCARD virtual std::string get_ee_link() const   = 0;
    MDV_NODISCARD virtual double      get_dt() const        = 0;

};  // class ParameterInterface


#endif  // MAGICIAN_MOTION_PLANNER_PARAMETER_INTERFACE_HPP
