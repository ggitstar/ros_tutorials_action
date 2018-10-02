#include "/opt/ros/kinetic/include/ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
//アクション目標状態ヘッダファイル
#include "ros_tutorials_action/FibonacciAction.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "action_client");
  //アクションクライアントの宣言
  actionlib::SimpleActionClient<ros_tutorials_action::FibonacciAction> ac("ros_tutorial_action",true);
  ROS_INFO("waiting for action server to start");
  ac.waitForServer();//アクションサーバが実行されるまで待機

  ROS_INFO("action server started sending goal");
  ros_tutorials_action::FibonacciGoal goal;
  goal.order=20;//アクション目標を指定
  ac.sendGoal(goal);//アクション目標を転送
  //アクション目標の達成に対するタイムリミットを設定
  bool finished_before_timeout=ac.waitForResult(ros::Duration(30.0));

  if(finished_before_timeout){
    actionlib::SimpleClientGoalState state=ac.getState();
    ROS_INFO("action finished: %s",state.toString().c_str());
  }
  else{
    ROS_INFO("action did not finish before timeout");
  }
  return 0;
}