#include "/opt/ros/kinetic/include/ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "ros_tutorials_action/FibonacciAction.h"

class FibonacciAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ros_tutorials_action::FibonacciAction> as_;
  //アクション名の変数
  std::string action_name_;
  //パブリッシュのためのアクションフィードバック及び結果のオブジェクト
  ros_tutorials_action::FibonacciFeedback feedback_;
  ros_tutorials_action::FibonacciResult result_;

public:
  //アクションサーバ初期化コンストラクタ
  FibonacciAction(std::string name) : as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
                                      action_name_(name)
  {
    as_.start();
  }
  ~FibonacciAction(void) {}
  //アクション目標(goal)メッセージを受信し、指定したアクションを実行する
  void executeCB(const ros_tutorials_action::FibonacciGoalConstPtr &goal)
  {
    ros::Rate r(1);
    bool success = true; //アクションの成功失敗

    //フィボナッチ数列の初期化
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    //アクション名、目標、数列のはじめの２つを出力
    ROS_INFO("%s:Executing,creating fibonacci sequence of order %i with seeds %i,%i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
    //アクションの内容
    for (int i = 1; i <= goal->order; i++)
    {
      //アクションクライアントから取り消しを確認
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s:Preempted", action_name_.c_str());
        as_.setPreempted(); //アクションの取り消しを知らせる
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i - 1]);
      as_.publishFeedback(feedback_);
      r.sleep();
    }
    if (success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s:Succed", action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "action_server");
  //フィボナッチ宣言アクション名"ros_tutorial_action"
  FibonacciAction fibonacci("ros_tutorial_action");
  //アクション受信の目標まで待つ
  ros::spin();
  return 0;
}