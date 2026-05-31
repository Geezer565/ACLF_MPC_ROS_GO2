#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>

#include <ocs2_legged_robot_adaptive/AdaptiveLeggedRobotInterface.h>

using namespace ocs2;
using namespace legged_robot;
using namespace legged_robot_adaptive;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  ::ros::init(argc, argv, robotName + "_adaptive_mpc");
  ::ros::NodeHandle nodeHandle;

  std::string taskFile, urdfFile, referenceFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/urdfFile", urdfFile);

  AdaptiveLeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(), interface.getOptimalControlProblem(),
                         interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc.getSolverPtr()->addSynchronizedModule(interface.getAdaptiveDisturbanceEstimatorPtr());

  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  return 0;
}
