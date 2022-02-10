/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>

#include <moveit_planning_helper/manage_trajectories.h>
#include <gtest/gtest.h>


namespace detail
{
  struct unwrapper
  {
    unwrapper(std::exception_ptr pe) : pe_(pe) {}

    operator bool() const
    {
      return bool(pe_);
    }

    friend auto operator<<(std::ostream& os, unwrapper const& u) -> std::ostream&
    {
      try
      {
          std::rethrow_exception(u.pe_);
          return os << "no exception";
      }
      catch(std::runtime_error const& e)
      {
          return os << "runtime_error: " << e.what();
      }
      catch(std::logic_error const& e)
      {
          return os << "logic_error: " << e.what();
      }
      catch(std::exception const& e)
      {
          return os << "exception: " << e.what();
      }
      catch(...)
      {
          return os << "non-standard exception";
      }
    }
    std::exception_ptr pe_;
  };
}

auto unwrap(std::exception_ptr pe)
{
  return detail::unwrapper(pe);
}

template<class F>
::testing::AssertionResult does_not_throw(F&& f)
{
  try
  {
     f();
     return ::testing::AssertionSuccess();
  }
  catch(...)
  {
     return ::testing::AssertionFailure() << unwrap(std::current_exception());
  }
};




std::shared_ptr<ros::NodeHandle> nh_private;
std::shared_ptr<ros::NodeHandle> nh;

// Declare a test
TEST(TestSuite, getTrajectoryFromParam)
{
  std::string full_path_to_trj = "/test/trj_ok";
  trajectory_msgs::JointTrajectory trj;
  std::string what;
  std::cout << "=============" << full_path_to_trj << "=============" << std::endl;
  EXPECT_TRUE(trajectory_processing::getTrajectoryFromParam(full_path_to_trj,trj,what));
  EXPECT_TRUE(what.size()==0);

  std::vector<std::string> warn{"/test/trj_warn_1", "/test/trj_warn_2"};
  for(const auto & w : warn )
  {
    full_path_to_trj = w;
    trajectory_msgs::JointTrajectory trj;
    std::string what;
    std::cout << "=============" << full_path_to_trj << "=============" << std::endl;
    EXPECT_TRUE(trajectory_processing::getTrajectoryFromParam(full_path_to_trj,trj,what));
    EXPECT_FALSE(what.size()==0);
    std::cout << what << std::endl;
  }

  std::vector<std::string> err{
    "/test/trj_err_0", "/test/trj_err_1", "/test/trj_err_2", "/test/trj_err_3", "/test/trj_err_4", "/test/trj_err_5",
      "test/trj_err_6", "/ns_that_does_not_exist/trj_err_7"};
  for(const auto & e : err )
  {
    full_path_to_trj = e;
    trajectory_msgs::JointTrajectory trj;
    std::string what;
    std::cout << "=============" << full_path_to_trj << "=============" << std::endl;
    bool ret = true;
    EXPECT_TRUE(does_not_throw([&]{ret = trajectory_processing::getTrajectoryFromParam(full_path_to_trj,trj,what);}));
    EXPECT_FALSE(ret);
    EXPECT_FALSE(what.size()==0);
    std::cout << what << std::endl;
  }
}



// Declare a test
TEST(TestSuite, getTrajectoryFromParamNh)
{
  trajectory_msgs::JointTrajectory trj;
  std::string trj_name = "trj_ok";
  std::cout << "=============" << nh_private->getNamespace()<< " - " << trj_name << " [TRUE EXPECTED] =============" << std::endl;
  EXPECT_TRUE( trajectory_processing::getTrajectoryFromParam(*nh_private,trj_name,trj) );

  trj_name = "/test/trj_ok";
  std::cout << "=============" << nh_private->getNamespace()<< " - " << trj_name << "[TRUE EXPECTED] =============" << std::endl;
  EXPECT_TRUE(trajectory_processing::getTrajectoryFromParam(*nh_private,trj_name,trj));

  trj_name = "trj_ok";
  std::cout << "=============" << nh->getNamespace()<< " - " << trj_name << "[FALSE EXPECTED] =============" << std::endl;
  EXPECT_FALSE( trajectory_processing::getTrajectoryFromParam(*nh,trj_name,trj) );

  trj_name = "/test/trj_ok";
  std::cout << "=============" << nh->getNamespace()<< " - " << trj_name << "[TRUE EXPECTED] =============" << std::endl;
  EXPECT_TRUE(trajectory_processing::getTrajectoryFromParam(*nh,trj_name,trj));


  std::vector<std::string> err{
    "/test/trj_err_0", "/test/trj_err_1", "/test/trj_err_2", "/test/trj_err_3", "/test/trj_err_4", "/test/trj_err_5",
      "test/trj_err_6", "trj_err_7"};
  for(const auto & e : err )
  {
    trj_name = e;
    std::string what;
    std::cout << "=============" << nh_private->getNamespace()<< " - " << trj_name << "[FALSE EXPECTED] =============" << std::endl;
    bool ret = true;
    EXPECT_TRUE(does_not_throw([&]{ret = trajectory_processing::getTrajectoryFromParam(*nh_private,trj_name,trj);}));
    EXPECT_FALSE(ret);

    std::cout << what << std::endl;
  }
  

}
// Declare a test
TEST(TestSuite, setTrajectoryFromParam)
{
}


// Declare a test
TEST(TestSuite, setTrajectoryFromParamNh)
{
  
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "manage_traejctory_test");

  nh_private.reset(new ros::NodeHandle("~"));
  nh.reset(new ros::NodeHandle());

  return RUN_ALL_TESTS();
}
