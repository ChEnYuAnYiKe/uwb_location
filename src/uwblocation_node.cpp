#include "uwb_location/uwblocation.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "uwb_location_node");
    ros::NodeHandle nh("~");

    Uwblocator uwblocator;

    uwblocator.init(nh);
    uwblocator.run();

	return 0;
}