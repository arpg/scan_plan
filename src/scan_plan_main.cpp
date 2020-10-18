#include "scan_plan.h"

// ***************************************************************************
scan_plan_class(ros::NodeHandle* nh)
{
  nh_ = nh;
  wait_for_params(nh_);
}

// ***************************************************************************
void scan_plan_class::wait_for_params(ros::NodeHandle* nh)
{
}
