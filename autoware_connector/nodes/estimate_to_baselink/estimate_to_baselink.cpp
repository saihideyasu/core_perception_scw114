#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <autoware_config_msgs/ConfigEstimateToBaselink.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_can_msgs/MicroBusPseudoOperation.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

class EstimateToBaselink
{
private:
	ros::NodeHandle nh_, p_nh_;

	ros::Publisher pub_current_pose_, pub_tiwst_, pub_localizer_pose_;
	ros::Subscriber sub_config_, sub_estimate_, sub_odometry_, sub_twist_, sub_waypoint_param_, sub_microbus_pseudo_operation_;

	tf::TransformBroadcaster broadcaster_;

	autoware_config_msgs::ConfigEstimateToBaselink config_;
	geometry_msgs::PoseWithCovarianceStamped estimate_;
	nav_msgs::Odometry odom_;
	autoware_msgs::VehicleCmd vehicle_cmd_;
	autoware_can_msgs::MicroBusPseudoOperation microbus_operation_;//microbusからの疑似操作情報
	double steer_correction_;//ステア制御倍率補正値

	void callbackConfig(const autoware_config_msgs::ConfigEstimateToBaselink &msg)
	{
		config_ = msg;
	}

	void callbackEstimate(const geometry_msgs::PoseWithCovarianceStamped &msg)
	{
		estimate_ = msg;
	}

	void callbackMicrobusPseudoOperation(const autoware_can_msgs::MicroBusPseudoOperation &msg)
	{
		Eigen::Quaterniond qua = Eigen::Quaterniond(estimate_.pose.pose.orientation.w, estimate_.pose.pose.orientation.x,
													estimate_.pose.pose.orientation.y, estimate_.pose.pose.orientation.z);
		double angle_red = msg.wheel_deg * M_PI / 180.0;
		Eigen::Quaterniond odom_qua = Eigen::Quaterniond(Eigen::AngleAxisd(angle_red * odom_.twist.twist.linear.x * steer_correction_ * 5, Eigen::Vector3d::UnitZ()));
		Eigen::Quaterniond new_qua = qua * odom_qua;
		estimate_.pose.pose.orientation.x = new_qua.x();
		estimate_.pose.pose.orientation.y = new_qua.y();
		estimate_.pose.pose.orientation.z = new_qua.z();
		estimate_.pose.pose.orientation.w = new_qua.w();

		microbus_operation_ = msg;
	}

	void callbackOdometry(const nav_msgs::Odometry &msg)
	{
		ros::Time nowtime = msg.header.stamp;
		ros::Time prev_time = odom_.header.stamp;
		ros::Duration ros_time_diff = nowtime - prev_time;
		double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;
		if(time_diff > 1.0) return;

		{
			Eigen::Vector3d po(estimate_.pose.pose.position.x, estimate_.pose.pose.position.y, estimate_.pose.pose.position.z);
			double roll, pitch, yaw;
			geometry_quat_to_rpy(roll, pitch, yaw, estimate_.pose.pose.orientation);
			Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
			Eigen::Vector3d po_rot = qua * po;
			po_rot += Eigen::Vector3d(msg.twist.twist.linear.x * time_diff * 2, 0, 0);
			Eigen::Quaterniond qua_rev = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
			Eigen::Vector3d po_move = qua_rev * po_rot;
			estimate_.pose.pose.position.x = po_move.x();
			estimate_.pose.pose.position.y = po_move.y();
			estimate_.pose.pose.position.z = po_move.z();
		}

		odom_ = msg;
	}

	//mpcから送られるvehicle_cmdトピックから車体角度を決めている
	void callbackVehicleCmd(const autoware_msgs::VehicleCmd &msg)
	{
		/*Eigen::Quaterniond qua = Eigen::Quaterniond(estimate_.pose.pose.orientation.w, estimate_.pose.pose.orientation.x,
													estimate_.pose.pose.orientation.y, estimate_.pose.pose.orientation.z);
		Eigen::Quaterniond odom_qua = Eigen::Quaterniond(Eigen::AngleAxisd(msg.ctrl_cmd.steering_angle/10.0 * steer_correction_, Eigen::Vector3d::UnitZ()));
		Eigen::Quaterniond new_qua = qua * odom_qua;
		estimate_.pose.pose.orientation.x = new_qua.x();
		estimate_.pose.pose.orientation.y = new_qua.y();
		estimate_.pose.pose.orientation.z = new_qua.z();
		estimate_.pose.pose.orientation.w = new_qua.w();
		
		vehicle_cmd_ = msg;*/
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam &msg)
	{
		if(msg.steer_correction > 0) steer_correction_ = msg.steer_correction;
	}
public:
	EstimateToBaselink(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
		, steer_correction_(1.0)
	{
		pub_current_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
		pub_tiwst_ = nh.advertise<geometry_msgs::TwistStamped>("/current_velocity", 10);
		pub_localizer_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);
		sub_config_ = nh_.subscribe("/config/estimate_to_baselink", 10, &EstimateToBaselink::callbackConfig, this);
		sub_estimate_ = nh_.subscribe("/initialpose", 10, &EstimateToBaselink::callbackEstimate, this);
		sub_odometry_ = nh_.subscribe("/vehicle/odom", 10, &EstimateToBaselink::callbackOdometry, this);
		sub_twist_ = nh_.subscribe("/vehicle_cmd", 10, &EstimateToBaselink::callbackVehicleCmd, this);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 10, &EstimateToBaselink::callbackWaypointParam, this);
		sub_microbus_pseudo_operation_ = nh_.subscribe("/microbus/pseudo_operation", 10, &EstimateToBaselink::callbackMicrobusPseudoOperation, this);

		estimate_.pose.pose.position.x = estimate_.pose.pose.position.y = estimate_.pose.pose.position.z = 0;
		estimate_.pose.pose.orientation.x = estimate_.pose.pose.orientation.y = estimate_.pose.pose.orientation.z = 0;
		estimate_.pose.pose.orientation.w = 1;

		odom_.header.stamp = ros::Time::now();
	}

	void pubBaseLink()
	{
		ros::Time nowtime = ros::Time::now();

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(estimate_.pose.pose.position.x, estimate_.pose.pose.position.y, config_.height) );
		tf::Quaternion q;
		q.setX(estimate_.pose.pose.orientation.x);  q.setY(estimate_.pose.pose.orientation.y);
		q.setZ(estimate_.pose.pose.orientation.z);  q.setW(estimate_.pose.pose.orientation.w);
		transform.setRotation(q);
		broadcaster_.sendTransform(tf::StampedTransform(transform, nowtime, "/map", "/base_link"));

		geometry_msgs::PoseStamped current_pose;
		current_pose.header.frame_id = "map";
		current_pose.header.stamp = nowtime;
		current_pose.pose = estimate_.pose.pose;
		current_pose.pose.position.z = config_.height;
		pub_current_pose_.publish(current_pose);
		pub_localizer_pose_.publish(current_pose);

		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "base_link";
		twist.header.stamp = nowtime;
		twist.twist.linear.x = odom_.twist.twist.linear.x;
		twist.twist.linear.y = twist.twist.linear.z = 0;
		twist.twist.angular.z = odom_.twist.twist.angular.z;
		twist.twist.angular.x = twist.twist.angular.y = 0;
		//twist.twist.linear.x = twist.twist.linear.y = twist.twist.linear.z = 0;
		//twist.twist.angular.x = twist.twist.angular.y = twist.twist.angular.z = 0;
		pub_tiwst_.publish(twist);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "estimate_to_baselink");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	EstimateToBaselink etb(nh, private_nh);

	ros::Rate rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
		etb.pubBaseLink();
		rate.sleep();
	}
	return 0;
}