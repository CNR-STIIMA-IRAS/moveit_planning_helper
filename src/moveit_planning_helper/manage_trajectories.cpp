#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <moveit_planning_helper/spline_interpolator.h>
#include <name_sorting/sort_trajectories.h>

#include <moveit_planning_helper/manage_trajectories.h>

template<typename T>
bool checkd(const std::vector<T>& in, const size_t& dim, const std::string& id, std::string& what)
{
  bool ret = in.size() == dim;
  if(!ret)
  {
    what += std::string(what.length()>0 ? "\n" : "") +
      "The vector '" + id + "' dimension is '"+ std::to_string(in.size()) + "' but '"+std::to_string(dim)+"' is expected";
  }
  return ret;
}

namespace trajectory_processing 
{

bool getTrajectoryFromParam(const std::string& full_path_to_trj, trajectory_msgs::JointTrajectory& trj, std::string& what)
{
  what = "";
  if((full_path_to_trj.length()==0)||(full_path_to_trj.front()!='/'))
  {
    what = "Error: The input 'full_path_to_trj' name is void or does not start with '/'";
    return false;
  }

  std::vector<std::string>          n;
  std::vector<double>               t;
  std::vector<std::vector<double>>  q, qd, qdd, eff;

  std::vector<std::string> params = 
  {"joint_names", "time_from_start", "positions", "velocities", "accelerations", "effort" };

  for(size_t i=0; i<params.size(); i++)
  {
    bool ok = true;
    std::string ns = full_path_to_trj+"/" + params.at(i);
    std::string w;
    switch(i)
    {
      case 0: 
        ok = rosparam_utilities::get(ns, n, w);
        break;
      case 1:
        rosparam_utilities::get(ns, t, w);
        break;
      case 2:
        ok = rosparam_utilities::get(ns, q, w);
        break;
      case 3:
        rosparam_utilities::get(ns, qd, w);
        break;
      case 4:
        rosparam_utilities::get(ns, qdd, w);
        break;
      case 5:
        rosparam_utilities::get(ns, eff, w);
        break;
    }
    what += (what.length()>0 && w.length()>0 ? "\n" : "") + w;
    if(!ok)
    {
      return false;
    }
  }

  if((q.size() == 0) || (q.at(0).size() == 0))
  {
    what += full_path_to_trj + "/positions is broken";
    return false;
  }
  size_t np = q.size();
  size_t dof = q.at(0).size();

  if((n.size()==0 || !checkd(n, dof, "joint_names", what))
  || (t.size()!=0 && !checkd(t, np , "time_from_start", what))
  || (qd.size()!=0 && !checkd(qd, np , "velocities", what))
  || (qdd.size()!=0 && !checkd(qdd, np , "accelerations", what))
  || (eff.size()!=0 && !checkd(eff, np , "effort", what) ))
  {
    return false;
  }

  std::map< std::string, std::vector<std::vector<double>>& > checks = 
    {{"position", q}, {"velocity", qd}, {"acceleration", qdd}, {"effort", eff}};

  for(const auto & cc : checks ) 
  {
    if(cc.second.size()!=0)
    {
      for(size_t i=0;i<np;i++)
      {
        if( !checkd( cc.second.at(i) , dof, cc.first +"#" + std::to_string(i), what))
        {
          return false;
        }
      }
    }
  }
  
  // TRJ MODIFICATION == == == == == == == == == == == == == == 
  trj.joint_names = n;
  trj.points.clear();
  trj.points.resize(np);

  for (size_t i=0;i<np;i++)
  {
    trj.points.at(i).positions = q.at(i);
    if (qd.size()>0)
    {
      trj.points.at(i).velocities = qd.at(i);
    }
    if (qdd.size()>0)
    {
      trj.points.at(i).accelerations = qdd.at(i);
    }
    if (eff.size()>0)
    {
      trj.points.at(i).effort = eff.at(i);
    }

    trj.points.at(i).time_from_start = (t.size()>0 ? ros::Duration(t.at(i)) : ros::Duration(0.0));
  }
  return true;
}


bool getTrajectoryFromParam(const ros::NodeHandle& nh, const std::string& trj_name, trajectory_msgs::JointTrajectory& trj)
{
  std::string what; 
  if((trj_name.length()==0))
  {
    what = "Error: The input 'trj_name' name is void";
    return false;
  }
  std::string full_ns = (trj_name.front()=='/') ? trj_name : nh.getNamespace() + "/" + trj_name;
  
  bool ret = getTrajectoryFromParam(full_ns, trj, what);
  if(!ret)
  {
    ROS_ERROR("%s", what.c_str());
  }
  return ret;
}

bool setTrajectoryToParam(const std::string& full_path_to_trj, const trajectory_msgs::JointTrajectory& trj, std::string& what)
{
  if (trj.points.size()==0)
  {
    what = "Trajectory with no points";
    return false;
  }

  std::vector<std::vector<double>> positions(trj.points.size());
  std::vector<std::vector<double>> velocities(trj.points.size());
  std::vector<std::vector<double>> accelerations(trj.points.size());
  std::vector<std::vector<double>> effort(trj.points.size());
  std::vector<double> time(trj.points.size());

  for (unsigned int i=0; i<trj.points.size(); i++)
  {
    positions.at(i) = trj.points.at(i).positions;
    velocities.at(i) = trj.points.at(i).velocities;
    accelerations.at(i) = trj.points.at(i).accelerations;
    effort.at(i) = trj.points.at(i).effort;
    time.at(i) = trj.points.at(i).time_from_start.toSec();
  }

  rosparam_utilities::set(full_path_to_trj+"/joint_names",trj.joint_names,what);
  rosparam_utilities::set(full_path_to_trj+"/time_from_start",time,what);


  if(!rosparam_utilities::set(full_path_to_trj+"/positions",positions,what)
  || !rosparam_utilities::set(full_path_to_trj+"/velocities",velocities,what)
  || !rosparam_utilities::set(full_path_to_trj+"/accelerations",accelerations,what)
  || !rosparam_utilities::set(full_path_to_trj+"/effort",effort,what))
    return false;

  return true;
}

bool setTrajectoryToParam(const ros::NodeHandle& nh, const std::string& trj_name, const trajectory_msgs::JointTrajectory& trj)
{
  std::string what; 
  bool ret = setTrajectoryToParam(nh.getNamespace()+"/"+ trj_name,trj, what);
  if(!ret)
  {
    ROS_ERROR("%s", what.c_str());
  }
  return ret;
}

bool checkCollisionBetweenTrajectories(const robot_trajectory::RobotTrajectory& trj1, const robot_trajectory::RobotTrajectory& trj2, const std::string& joined_group_name)
{

  if (!trj1.getGroupName().compare(trj2.getGroupName()))
  {
    ROS_ERROR("Robot trajectories should be of two different move gruop");
    return false;
  }
  
  assert(0);
  
  return false;
}

double computeTrajectoryLength(const trajectory_msgs::JointTrajectory &trj)
{
  if (trj.points.size()<1)
    return 0;

  double length=0;
  for (unsigned int idx=1;idx<trj.points.size();idx++)
  {
    std::vector<double> last_pos=trj.points.at(idx-1).positions;
    std::vector<double> pos=trj.points.at(idx).positions;
    double squared_segment_length=0;
    for (unsigned int iax=0;iax<pos.size();iax++)
    {
      squared_segment_length+=std::pow(pos.at(iax)-last_pos.at(iax),2);
    }
    length+=std::sqrt(squared_segment_length);
  }
  return length;
}
}
