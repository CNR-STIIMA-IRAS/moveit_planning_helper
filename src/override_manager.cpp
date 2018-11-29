#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64.h>

bool is_paused;
bool setPauseResume(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  is_paused=req.data;
  res.success=true;
  if (req.data)
    ROS_INFO("stop movement");
  else 
    ROS_INFO("resume movement");
  return true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "override_manager");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  ros::Rate lp(1000);
  spinner.start();
  
  
  is_paused=false;
  ros::ServiceServer pause_srv=nh.advertiseService("/pause",setPauseResume);
  ros::Publisher safe_ovr_pub=nh.advertise<std_msgs::Int64>("/safe_ovr_2",1);
  
  std_msgs::Int64 safe_ovr_msg;
  
  ros::Time t0=ros::Time::now();
  
  double ovr=100;
  double ovr_limited=100;
  double max_delta=1;
  while (ros::ok())
  {
    ROS_INFO_THROTTLE(300,"Override Manager:\n==> /pause [std_srvs::SetBool] is a service to pause [true]/resume [false] the movement.");
    ovr = 100*!is_paused;
    
    if (ovr>ovr_limited)
      ovr_limited=std::min(100.0,ovr_limited+max_delta);
    else if (ovr<ovr_limited)
      ovr_limited=std::max(0.0,ovr_limited-max_delta);
    safe_ovr_msg.data=ovr_limited;
    safe_ovr_pub.publish(safe_ovr_msg);
    lp.sleep();
  }
  
  ros::shutdown();
  return 0;
}
