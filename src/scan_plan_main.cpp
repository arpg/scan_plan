#include "scan_plan.h"

// ***************************************************************************
scan_plan::scan_plan(ros::NodeHandle* nh)
{
  nh_ = nh;
  wait_for_params(nh_);
}

// ***************************************************************************
void scan_plan::wait_for_params(ros::NodeHandle* nh)
{
}
