#include "scan_plan.h"

using namespace std;
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_plan_node");
	ros::NodeHandle nh(ros::this_node::getName());
	
	scan_plan scanPlan(&nh);
	
	ros::spin();

	return 0;
}
