#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

//geometry_msgs/Quaternionからroll,pitch,yawを取得
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

tf::Pose createMatrix(const geometry_msgs::PoseStamped& pose)
{
	tf::Pose ret;
	ret.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
	ret.setRotation(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w));
	return ret;
}

class LocalizerTransforrm
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Publisher pub_transform_;
	ros::Subscriber sub_pose1_, sub_pose2_;

	geometry_msgs::PoseStamped pose1_, pose2_;

	void callbackPose1(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		pose1_ = *msg;
	}

	void callbackPose2(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		pose2_ = *msg;
	}
public:
	LocalizerTransforrm(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		std::string pose_name1, pose_name2;
		pnh_.param<std::string>("pose_name1", pose_name1, "/ndt_pose");
		pnh_.param<std::string>("pose_name2", pose_name2, "/RTK_gnss_pose");
		pub_transform_ = nh_.advertise<geometry_msgs::PoseStamped>("/localizer_transform", 1);
		sub_pose1_ = nh_.subscribe(pose_name1, 10, &LocalizerTransforrm::callbackPose1, this);
		sub_pose2_ = nh_.subscribe(pose_name2, 10, &LocalizerTransforrm::callbackPose2, this);
	}

	void transformCalc()
	{
		/*Eigen::Translation3f tr_pose1(pose1_.pose.position.x, pose1_.pose.position.y, pose1_.pose.position.z);
		Eigen::Translation3f tr_pose2(pose2_.pose.position.x, pose2_.pose.position.y, pose2_.pose.position.z);

		double roll1, pitch1, yaw1;
		geometry_quat_to_rpy(roll1, pitch1, yaw1, pose1_.pose.orientation);
		Eigen::AngleAxisf rot_roll1(roll1, Eigen::Vector3f::UnitX());  // rot: rotation
		Eigen::AngleAxisf rot_pitch1(pitch1, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_yaw1(yaw1, Eigen::Vector3f::UnitZ());
		Eigen::Matrix4f mat1 = (tr_pose1 * rot_yaw1 * rot_pitch1 * rot_roll1).matrix();
		double roll2, pitch2, yaw2;
		geometry_quat_to_rpy(roll2, pitch2, yaw2, pose2_.pose.orientation);
		Eigen::AngleAxisf rot_roll2(roll2, Eigen::Vector3f::UnitX());  // rot: rotation
		Eigen::AngleAxisf rot_pitch2(pitch2, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf rot_yaw2(yaw2, Eigen::Vector3f::UnitZ());
		Eigen::Matrix4f mat2 = (tr_pose2 * rot_yaw2 * rot_pitch2 * rot_roll2).matrix();

		Eigen::Affine3f aff(mat1.inverse() * mat2);*/

		tf::Pose mat1 = createMatrix(pose1_);
		tf::Pose mat2 = createMatrix(pose2_);
		std::cout << mat1.getOrigin().getX() << "," << mat1.getOrigin().getY() << "," << mat1.getOrigin().getZ() << std::endl;
		std::cout << mat1.getRotation().getX() << "," << mat1.getRotation().getY() << "," << mat1.getRotation().getZ() << "," << mat1.getRotation().getZ() << std::endl;
		std::cout << mat2.getOrigin().getX() << "," << mat2.getOrigin().getY() << "," << mat2.getOrigin().getZ() << std::endl;
		std::cout << mat2.getRotation().getX() << "," << mat2.getRotation().getY() << "," << mat2.getRotation().getZ() << "," << mat2.getRotation().getZ() << std::endl;
		tf::Transform transform = mat1.inverse() * mat2;
		ros::Time time = (pose1_.header.stamp < pose2_.header.stamp) ? pose1_.header.stamp : pose2_.header.stamp;

		geometry_msgs::PoseStamped ret;
		ret.header.stamp = time;
		ret.pose.position.x = transform.getOrigin().getX();
		ret.pose.position.y = transform.getOrigin().getY();
		ret.pose.position.z = transform.getOrigin().getZ();
		ret.pose.orientation.x = transform.getRotation().getX();
		ret.pose.orientation.y = transform.getRotation().getY();
		ret.pose.orientation.z = transform.getRotation().getZ();
		ret.pose.orientation.w = transform.getRotation().getW();
		pub_transform_.publish(ret);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "localizer_transform");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	LocalizerTransforrm lt(nh, pnh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		lt.transformCalc();
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}