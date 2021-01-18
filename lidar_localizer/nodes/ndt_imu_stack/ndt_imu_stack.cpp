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

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

class NdtImuStack
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Publisher pub_imu_pose_stack_, pub_ndt_imu_yaw_;
	ros::Subscriber sub_imu_, sub_ndt_pose_;

	geometry_msgs::PoseStamped stack_pose_;
	ros::Time prev_imu_time_;

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

		/*Eigen::Translation3f xyz(stack_pose_.pose.position.x, stack_pose_.pose.position.y, stack_pose_.pose.position.z);//translation
		double roll, pitch, yaw;
		tf::Quaternion qua;
		tf::quaternionMsgToTF(stack_pose_.pose.orientation, qua);
		tf::Matrix3x3(qua).getRPY(roll, pitch, yaw);
		Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());  //rotation
		Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
		Eigen::Matrix4f mat_pose = (xyz * rot_z * rot_y * rot_x).matrix();

		Eigen::Translation3f xyz_imu(-msg.linear_acceleration.x*td*10, -msg.linear_acceleration.y*td*10,0);//translation
		Eigen::AngleAxisf rot_x_imu(0, Eigen::Vector3f::UnitX());  //rotation
		Eigen::AngleAxisf rot_y_imu(0, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_z_imu(-msg.angular_velocity.z * td *1.5, Eigen::Vector3f::UnitZ());
		Eigen::Matrix4f mat_imu = (xyz_imu * rot_z_imu * rot_y_imu * rot_x_imu).matrix();

		Eigen::Matrix4f mat_new_pose = mat_pose * mat_imu;
		tf::Matrix3x3 mat_b;  // base_link
		mat_b.setValue(static_cast<double>(mat_new_pose(0, 0)), static_cast<double>(mat_new_pose(0, 1)), static_cast<double>(mat_new_pose(0, 2)),
                   static_cast<double>(mat_new_pose(1, 0)), static_cast<double>(mat_new_pose(1, 1)), static_cast<double>(mat_new_pose(1, 2)),
                   static_cast<double>(mat_new_pose(2, 0)), static_cast<double>(mat_new_pose(2, 1)), static_cast<double>(mat_new_pose(2, 2)));

		// Update ndt_pose
		stack_pose_.pose.position.x = mat_new_pose(0, 3);
		stack_pose_.pose.position.y = mat_new_pose(1, 3);
		stack_pose_.pose.position.z = mat_new_pose(2, 3);*/


		/*double roll_new, pitch_new, yaw_new;
		mat_new_pose.getRPY(roll_new, pitch_new, yaw_new, 1);
		geometry_quat_to_rpy(roll_new, pitch_new, yaw_new,stack_pose_.pose.orientation);*/
		
		
		
		/*stack_pose_.pose.position.x -= msg.linear_acceleration.x * td*100;
		stack_pose_.pose.position.y -= msg.linear_acceleration.y * td*100;*/

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
		imu_pub_data.x = 0;
		imu_pub_data.y = 0;
		imu_pub_data.z = 0;
		pub_ndt_imu_yaw_.publish(imu_pub_data);

		stack_pose_.header.stamp = msg.header.stamp;
		pub_imu_pose_stack_.publish(stack_pose_);
		std::cout << msg.linear_acceleration.x << "," << msg.linear_acceleration.y << std::endl;
		prev_imu_time_ = msg.header.stamp;
	}

	void callbackPose(const geometry_msgs::PoseStamped &msg)
	{
		stack_clear();
		//stack_pose_.header.stamp = msg.header.stamp;
		//pub_imu_pose_stack_.publish(stack_pose_);
	}
public:
	NdtImuStack(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		prev_imu_time_ = ros::Time::now();
		stack_clear();

		pub_imu_pose_stack_ = nh_.advertise<geometry_msgs::PoseStamped>("/ndt_imu_pose_stack", 1);
		pub_ndt_imu_yaw_ = nh_.advertise<autoware_msgs::ImuValues>("/ndt_imu_values", 1);
		sub_imu_ = nh_.subscribe("/imu/data", 10, &NdtImuStack::callbackImu, this);
		sub_ndt_pose_ = nh_.subscribe("/ndt_pose", 10, &NdtImuStack::callbackPose, this);
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