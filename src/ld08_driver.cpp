#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "transform.h"

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59/180000)


int main(int argc , char **argv)
{
	LiPkg * pkg;
	pkg = new LD08_LiPkg; 
	int32_t ver=8;

#if 0	 
    /* test code */
	#ifdef USE_SLBF
		std::cout << "USE_SLBF"<<std::endl; 
	#endif
	#ifdef USE_SLBI
		std::cout << "USE_SLBI"<<std::endl; 
	#endif
#endif

	char product_ver[5]={0};   				/*production version*/
	strcpy(product_ver,"LD08");
	ros::init(argc, argv, product_ver);
	ros::NodeHandle nh;                    /* create a ROS Node */
	char topic_name[20]={0};
	strcat(topic_name,product_ver);
	strcat(topic_name,"/scan");
	ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1); /*create a ROS topic */

    CmdInterfaceLinux cmd_port(ver);
    std::vector<std::pair<std::string, std::string> > device_list;
    std::string port_name;
    cmd_port.GetCmdDevices(device_list);
    for (auto n : device_list)
    {
        std::cout << n.first << "    " << n.second << std::endl;
        if(strstr(n.second.c_str(),"CP2102"))
        {
            port_name = n.first;
        }
    }

	if(port_name.empty() == false)
	{
		std::cout<<"FOUND LiDAR_" <<  product_ver << " @port :" << port_name << std::endl;
		cmd_port.SetReadCallback([&pkg](const char *byte, size_t len) {
			if(pkg->Parse((uint8_t*)byte, len))
			{
				pkg->AssemblePacket();  
			}
		});
		cmd_port.Open(port_name);
		sensor_msgs::LaserScan scan;
		scan.header.stamp = ros::Time::now();
		scan.header.frame_id = "base_scan";
		scan.range_min = 0.0;
		scan.range_max = 100.0;

		while (ros::ok())
		{
			if (pkg->IsFrameReady())
			{
				FrameData data = pkg->GetFrameData();
				scan.angle_min = ANGLE_TO_RADIAN(data.angle_min);
				scan.angle_max = ANGLE_TO_RADIAN(data.angle_max);
				scan.angle_increment = (scan.angle_max - scan.angle_min)/data.len;
				scan.ranges.resize(data.len);
				scan.intensities.resize(data.len);
				for(int i=0;i<data.len;i++)
				{
					scan.ranges[i] = data.distance[i]/1000.f;
					scan.intensities[i] = data.intensities[i];
				}
				scan.header.stamp = ros::Time::now();
				lidar_pub.publish(scan);
			}
		}
	}
	else
	{
		std::cout<<"Can't find LiDAR"<< argv[1] << std::endl;
	}


    return 0;
}

