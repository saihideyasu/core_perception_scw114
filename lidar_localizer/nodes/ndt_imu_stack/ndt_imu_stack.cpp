#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float64.h>
#include <autoware_msgs/ImuValues.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

class NdtImuStack
{
private:
	const double LOWPASS_ALPHA = 0.8;//ローパスフィルタのウェイト

	ros::NodeHandle nh_, pnh_;
	ros::Publisher pub_imu_pose_stack_, pub_ndt_imu_yaw_;
	ros::Subscriber sub_imu_, sub_ndt_pose_, sub_velocity_, sub_initialpose_;

	geometry_msgs::PoseStamped stack_pose_;//imuから計算した位置、角度情報の累積  ndt_pose更新時にリセットされる
	ros::Time prev_imu_time_;//前回のimu処理時間
	geometry_msgs::TwistStamped current_vel_;//外部トピックからsubscribeされる現在の速度  距離計算に使用
	ros::Timer initialpose_timer_;//rvizの2D pose estimateで送られる/initialposeトピックのsubscribe時にセットされるタイマー  ndtの位置合わせを安定させるために、指定秒数imu情報をリセットする
	bool initialpose_flag_;

	double gravity_x_, gravity_y_, gravity_z_;//ローパスフィルタで作成した重力加速度

	void stack_clear()
	{
		stack_pose_.pose.position.x = stack_pose_.pose.position.y = stack_pose_.pose.position.z = 0;
		stack_pose_.pose.orientation.x = stack_pose_.pose.orientation.y = stack_pose_.pose.orientation.z = 0;
		stack_pose_.pose.orientation.w = 1;
	}

	void callbackImu(const sensor_msgs::Imu &msg)
	{
		ros::Duration time_diff = msg.header.stamp - prev_imu_time_;
		double td = time_diff.sec + time_diff.nsec * 1E-9;

		//ローパスフィルタで位置情報から重力加速度を除去する。
		gravity_x_ = LOWPASS_ALPHA * gravity_x_ + (1 - LOWPASS_ALPHA) * msg.linear_acceleration.x;
		gravity_y_ = LOWPASS_ALPHA * gravity_y_ + (1 - LOWPASS_ALPHA) * msg.linear_acceleration.y;
		gravity_z_ = LOWPASS_ALPHA * gravity_z_ + (1 - LOWPASS_ALPHA) * msg.linear_acceleration.z;
		double acc_x = msg.linear_acceleration.x - gravity_x_;
		double acc_y = msg.linear_acceleration.y - gravity_y_;
		double acc_z = msg.linear_acceleration.z - gravity_z_;

		//変化した移動量の計算
		double x_vel_imu = current_vel_.twist.linear.x - acc_x / 2.0 * td;
		double y_vel_imu = current_vel_.twist.linear.y - acc_y / 2.0 * td;
		double z_vel_imu = current_vel_.twist.linear.z - acc_z / 2.0 * td;
		double x_imu = x_vel_imu * td;
		double y_imu = y_vel_imu * td;
		double z_imu = z_vel_imu * td;

		//変化した角度量の計算		
		double yaw_imu = -msg.angular_velocity.z * td;
		tf::Quaternion quat_imu = tf::createQuaternionFromYaw(yaw_imu);
		tf::Quaternion quat_orig;
		tf::quaternionMsgToTF(stack_pose_.pose.orientation, quat_orig);
		tf::Quaternion quat_new = quat_orig * quat_imu;
		tf::quaternionTFToMsg(quat_new, stack_pose_.pose.orientation);
		//tf::quaternionTFToMsg(quat_imu, stack_pose_.pose.orientation);

		double pub_roll, pub_pitch, pub_yaw;
		geometry_quat_to_rpy(pub_roll, pub_pitch, pub_yaw, stack_pose_.pose.orientation);
		autoware_msgs::ImuValues imu_pub_data;
		imu_pub_data.header.stamp = msg.header.stamp;
		imu_pub_data.roll_deg = 0;
		imu_pub_data.pitch_deg = 0;
		imu_pub_data.yaw_deg = pub_yaw * 180 / M_PI;
		imu_pub_data.x = x_vel_imu * 1000;
		imu_pub_data.y = 0;//y_vel_imu * 1000;
		imu_pub_data.z = 0;//z_vel_imu * 1000;
		pub_ndt_imu_yaw_.publish(imu_pub_data);

		stack_pose_.header.stamp = msg.header.stamp;
		pub_imu_pose_stack_.publish(stack_pose_);
		std::cout << msg.linear_acceleration.x << "," << msg.linear_acceleration.y << std::endl;
		prev_imu_time_ = msg.header.stamp;
	}

	void callbackPose(const geometry_msgs::PoseStamped &msg)
	{
		stack_clear();
	}

	void callbackVelocity(const geometry_msgs::TwistStamped &msg)
	{
		if(initialpose_flag_ == true)
		{
			current_vel_.header.stamp = msg.header.stamp;
			current_vel_.twist.linear.x = current_vel_.twist.linear.y = current_vel_.twist.linear.z = 0;
			current_vel_.twist.angular.x = current_vel_.twist.angular.y = current_vel_.twist.angular.z = 0;
		}
		else
		{
			current_vel_ = msg;
		}
	}

	void callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped &msg)
	{
		initialpose_flag_ = true;
		initialpose_timer_.start();
	}

	void initialposeTimerCallback(const ros::TimerEvent& e)
	{
		initialpose_flag_ = false;
		initialpose_timer_.stop();
	}
public:
	NdtImuStack(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
		, initialpose_flag_(false)
		, gravity_x_(0), gravity_y_(0), gravity_z_(0)
	{
		prev_imu_time_ = ros::Time::now();
		stack_clear();

		current_vel_.twist.linear.x = current_vel_.twist.linear.y = current_vel_.twist.linear.z = 0;
		current_vel_.twist.angular.x = current_vel_.twist.angular.y = current_vel_.twist.angular.z = 0;

		pub_imu_pose_stack_ = nh_.advertise<geometry_msgs::PoseStamped>("/ndt_imu_pose_stack", 1);
		pub_ndt_imu_yaw_ = nh_.advertise<autoware_msgs::ImuValues>("/ndt_imu_values", 1);
		sub_imu_ = nh_.subscribe("/imu/data", 10, &NdtImuStack::callbackImu, this);
		sub_ndt_pose_ = nh_.subscribe("/ndt_pose", 10, &NdtImuStack::callbackPose, this);
		sub_velocity_ = nh_.subscribe("/current_velocity", 10, &NdtImuStack::callbackVelocity, this);
		sub_initialpose_ = nh_.subscribe("/initialpose", 10, &NdtImuStack::callbackInitialpose, this);
		initialpose_timer_ = nh_.createTimer(ros::Duration(0.2), &NdtImuStack::initialposeTimerCallback, this);
		initialpose_timer_.stop();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ndt_imu_stack");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	NdtImuStack nis(nh, private_nh);
	ros::spin();
	return 0;
}