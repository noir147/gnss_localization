#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Pose.h"

class GnssLocalizationNode{
private:
  std::string base_frame_id_;
  std::string odom_frame_id_;
  std::string global_frame_id_;

  tf::TransformListener tf_odom_listener_;

  geometry_msgs::PoseStamped base_global_pose_;
  geometry_msgs::PoseStamped start_base_global_pose_;

  geometry_msgs::Pose start_pose_;

  bool is_get_global_pose_;
  bool is_no_get_base_pose_;

  ros::NodeHandle private_nh_;
  ros::NodeHandle nh_;

  ros::Subscriber change_global_pose_sub_;
  ros::Publisher gnss_localization_pose_pub_;

  std::string old_localization_mode_;

public:
  GnssLocalizationNode()
    : private_nh_("~")
  {
    old_localization_mode_ = "";

    private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));

    change_global_pose_sub_ = nh_.subscribe("/base_global_pose", 1000, &GnssLocalizationNode::baseGlobalPoseCallback, this);

    gnss_localization_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/gnss_localization_pose", 1000);

    is_get_global_pose_ = false;

    while(base_global_pose_.header.frame_id == ""){
      ros::spinOnce();

      ROS_ERROR("No get localization pose");

      if(!ros::ok()){
        exit(-1);
      }
      ros::Duration(1).sleep();
    }

    is_get_global_pose_ = false;
    is_no_get_base_pose_ = true;
  }

  ~GnssLocalizationNode()
  {

  }

  void run()
  {
    ROS_INFO("Start");

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
      ros::spinOnce();

      geometry_msgs::PoseStamped msg;

      //gnss

      //
      std::string localization_mode;

      nh_.param("localization_mode", localization_mode, std::string(""));

      if(is_get_global_pose_ && localization_mode == "odom" && old_localization_mode_ != "odom"){
        start_base_global_pose_ = base_global_pose_;

        //start_pose_.position.x = odom_x;
        //start_pose_.position.y = odom_y;
        //tf::quaternionTFToMsg(tf::createQuaternionFromYaw(odom_yaw), start_pose_.orientation);

        is_get_global_pose_  = false;
        is_no_get_base_pose_ = false;

        ROS_INFO("Get global_pose");
      }

      old_localization_mode_ = localization_mode;

      //ジャイロデータ取得予定
      //現在はodomを使用

      if(!is_no_get_base_pose_){
        double start_yaw = tf::getYaw(start_pose_.orientation);
        double base_yaw  = tf::getYaw(start_base_global_pose_.pose.orientation);

        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = global_frame_id_;
        //msg.pose.position.x = start_base_global_pose_.pose.position.x + odom_x - start_pose_.position.x;
        //msg.pose.position.y = start_base_global_pose_.pose.position.y + odom_y - start_pose_.position.y;
        //tf::quaternionTFToMsg(tf::createQuaternionFromYaw(base_yaw + odom_yaw - start_yaw), msg.pose.orientation);

        gnss_localization_pose_pub_.publish(msg);
      }else{
        gnss_localization_pose_pub_.publish(base_global_pose_);
      }

      loop_rate.sleep();
    }
  }

  void baseGlobalPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    base_global_pose_ = *msg;

    is_get_global_pose_ = true;
  }

  bool getOdomPos(const std::string target_frame,
                  const std::string source_frame,
                  double &x, double &y, double &yaw)
  {
    try
    {
      tf::StampedTransform trans;
      tf_odom_listener_.waitForTransform(target_frame, source_frame,
                                         ros::Time(0), ros::Duration(0.5));
      tf_odom_listener_.lookupTransform(target_frame, source_frame,
                                        ros::Time(0), trans);
      x = trans.getOrigin().x();
      y = trans.getOrigin().y();
      yaw = tf::getYaw(trans.getRotation());

      return true;
    }
    catch(tf::TransformException &e)
    {
      ROS_WARN("%s", e.what());
      return false;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnss_localization_node");

  GnssLocalizationNode gnssLocalizationNode;

  gnssLocalizationNode.run();

  return 0;
}
