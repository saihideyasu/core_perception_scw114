#include <ros/ros.h>
#include <istream>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/GnssStandardDeviation.h>
#include <autoware_msgs/LocalizerMatchStat.h>
#include <autoware_msgs/WaypointParam.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_config_msgs/ConfigLocalizerSwitch.h>

const int max_localizer_count = 2;
const int SYNC_FRAMES = 10;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped, autoware_msgs::NDTStat>
	NdtlocalizerSync;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped, autoware_msgs::GnssStandardDeviation>
	RTKlocalizerSync;

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped>
	VelocitylocalizerSync;

//typedef void (* Localizer_Publisher)(geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, geometry_msgs::PoseStamped);

class LocalizerSwitch;

tf::TransformListener *tf_listener;

class TopicList
{
private:
	ros::NodeHandle nh_, private_nh_;
	tf::TransformBroadcaster trans_broad;

	message_filters::Subscriber<geometry_msgs::PoseStamped>             *base_link_pose_sub_;
	message_filters::Subscriber<geometry_msgs::TwistStamped>            *estimate_twist_sub_;
	message_filters::Subscriber<geometry_msgs::PoseStamped>             *localizer_pose_sub_;
	message_filters::Subscriber<autoware_msgs::NDTStat>                 *ndt_status_sub_;
	message_filters::Subscriber<autoware_msgs::GnssStandardDeviation> *gnss_deviation_sub_;
	message_filters::Synchronizer<NdtlocalizerSync> *sync_ndt_;
	message_filters::Synchronizer<RTKlocalizerSync> *sync_RTK_;
	message_filters::Synchronizer<VelocitylocalizerSync> *sync_velocity_;

	ros::Publisher pub_current_pose_, pub_twist_pose_, pub_localizer_pose_;

	//class ID
	int classID;
	//Use localizer topic group
	std::string base_link_pose_topic_, estimate_twist_topic_, localizer_pose_topic_;
	//localizer's approach
	int approach_;
	//status topic to use when approach is ndt
	std::string ndt_status_topic_;
	//status topic to use when approach is GNSS(RTK)
	std::string gnss_deviation_topic_;
	//fusion select flag
	int fusion_select_;
	//yaw_correction
	double yaw_correction_;

	//baselink data
	geometry_msgs::PoseStamped baselink_;
	//twist data
	geometry_msgs::TwistStamped twist_;
	//localizer data
	geometry_msgs::PoseStamped localizer_;

	void posedata_write(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
						const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
						const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg)
	{
		tf::Quaternion pose_orien;
		tf::quaternionMsgToTF(base_link_pose_msg->pose.orientation, pose_orien);
		std::cout << "yaw col : " << yaw_correction_ << std::endl;
		tf::Quaternion hosei = tf::createQuaternionFromYaw(yaw_correction_ * M_PI /180.0);
		tf::Quaternion math_orie = pose_orien * hosei;

		baselink_.header.frame_id = base_link_pose_msg->header.frame_id;
		baselink_.header.stamp = base_link_pose_msg->header.stamp;
		baselink_.header.seq = base_link_pose_msg->header.seq;
		baselink_.pose.position.x = base_link_pose_msg->pose.position.x;
		baselink_.pose.position.y = base_link_pose_msg->pose.position.y;
		baselink_.pose.position.z = base_link_pose_msg->pose.position.z;
		baselink_.pose.orientation.x = math_orie.getX();//base_link_pose_msg->pose.orientation.x;
		baselink_.pose.orientation.y = math_orie.getY();//base_link_pose_msg->pose.orientation.y;
		baselink_.pose.orientation.z = math_orie.getZ();//base_link_pose_msg->pose.orientation.z;
		baselink_.pose.orientation.w = math_orie.getW();//base_link_pose_msg->pose.orientation.w;

		twist_.header.frame_id = estimate_twist_msg->header.frame_id;
		twist_.header.stamp = estimate_twist_msg->header.stamp;
		twist_.header.seq = estimate_twist_msg->header.seq;
		twist_.twist.linear.x = estimate_twist_msg->twist.linear.x;
		twist_.twist.linear.y = estimate_twist_msg->twist.linear.y;
		twist_.twist.linear.z = estimate_twist_msg->twist.linear.z;
		twist_.twist.angular.x = estimate_twist_msg->twist.angular.x;
		twist_.twist.angular.y = estimate_twist_msg->twist.angular.y;
		twist_.twist.angular.z = estimate_twist_msg->twist.angular.z;

		localizer_.header.frame_id = localizer_pose_msg->header.frame_id;
		localizer_.header.stamp = localizer_pose_msg->header.stamp;
		localizer_.header.seq = localizer_pose_msg->header.seq;
		localizer_.pose.position.x = localizer_pose_msg->pose.position.x;
		localizer_.pose.position.y = localizer_pose_msg->pose.position.y;
		localizer_.pose.position.z = localizer_pose_msg->pose.position.z;
		localizer_.pose.orientation.x = localizer_pose_msg->pose.orientation.x;
		localizer_.pose.orientation.y = localizer_pose_msg->pose.orientation.y;
		localizer_.pose.orientation.z = localizer_pose_msg->pose.orientation.z;
		localizer_.pose.orientation.w = localizer_pose_msg->pose.orientation.w;
	}

	void NdtlocalizerCallback(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
									const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
									const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg,
									const autoware_msgs::NDTStatConstPtr &ndt_stConstPtratus_msg)
	{
		//std::cout << "aaa\n" << std::flush;
		posedata_write(base_link_pose_msg, estimate_twist_msg, localizer_pose_msg);

		switch(fusion_select_)
		{
		case 1:
			{
				pose_topic_publish();
				break;
			}
		}
	}

	void RTKlocalizerCallback(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
									const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
									const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg,
									const autoware_msgs::GnssStandardDeviationConstPtr &gnss_deviation_msg)
	{
		//std::cout << "bbb\n" << std::flush;
		posedata_write(base_link_pose_msg, estimate_twist_msg, localizer_pose_msg);

		switch(fusion_select_)
		{
		case 1:
			{
				pose_topic_publish();
				break;
			}
		}
	}

	void VelocitylocalizerCallback(const geometry_msgs::PoseStampedConstPtr &base_link_pose_msg,
									const geometry_msgs::TwistStampedConstPtr &estimate_twist_msg,
									const geometry_msgs::PoseStampedConstPtr &localizer_pose_msg)
	{
		posedata_write(base_link_pose_msg, estimate_twist_msg, localizer_pose_msg);
		switch(fusion_select_)
		{
		case 1:
			{
				pose_topic_publish();
				break;
			}
		}
	}
public:
	TopicList(ros::NodeHandle nh, ros::NodeHandle private_nh,
			  std::string baseLinkPoseTopic, std::string estimateTwistTopic, std::string localizerPoseTopic,
			  int approachFlag, std::string ndtStatusTopic, std::string gnssDeviationTopic)
		: nh_(nh)
		, private_nh_(private_nh)
		, fusion_select_(-1)
	{
		static int classID_counter = 0;
		classID = classID_counter;   classID_counter++;
		base_link_pose_topic_ = baseLinkPoseTopic;   estimate_twist_topic_ = estimateTwistTopic;
		localizer_pose_topic_ = localizerPoseTopic;
		approach_ = approachFlag;
		ndt_status_topic_ = ndtStatusTopic;
		gnss_deviation_topic_ = gnssDeviationTopic;

		pub_current_pose_   = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
		pub_twist_pose_     = nh_.advertise<geometry_msgs::TwistStamped>("/current_velocity", 10);
		pub_localizer_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);
	}

	std::string get_base_link_pose_topic() {return base_link_pose_topic_; }
	std::string get_estimate_twist_topic() {return estimate_twist_topic_; }
	std::string get_localizer_pose_topic() {return localizer_pose_topic_; }
	int         get_approach() {return approach_; }
	std::string get_ndt_status_topic() {return ndt_status_topic_; }
	std::string get_gnss_deviation_topic() {return gnss_deviation_topic_; }

	void callback_run()
	{
		// subscriber
		base_link_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, base_link_pose_topic_, 10);
		estimate_twist_sub_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, estimate_twist_topic_, 10);
		localizer_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, localizer_pose_topic_, 10);
		switch(approach_)
		{
		case 0://ndt
			{
				ndt_status_sub_ = new message_filters::Subscriber<autoware_msgs::NDTStat>(nh_, ndt_status_topic_, 10);
				sync_ndt_ = new message_filters::Synchronizer<NdtlocalizerSync>(NdtlocalizerSync(SYNC_FRAMES),
								  *base_link_pose_sub_, *estimate_twist_sub_, *localizer_pose_sub_, *ndt_status_sub_);
				sync_ndt_->registerCallback(boost::bind(&TopicList::NdtlocalizerCallback, this, _1, _2, _3, _4));
				break;
			}
		case 1://gnss(RTK)
			{
				gnss_deviation_sub_ = new message_filters::Subscriber<autoware_msgs::GnssStandardDeviation>(nh_, gnss_deviation_topic_, 10);
				sync_RTK_ = new message_filters::Synchronizer<RTKlocalizerSync>(RTKlocalizerSync(SYNC_FRAMES),
								  *base_link_pose_sub_, *estimate_twist_sub_, *localizer_pose_sub_, *gnss_deviation_sub_);
				sync_RTK_->registerCallback(boost::bind(&TopicList::RTKlocalizerCallback, this, _1, _2, _3, _4));
				break;
			}
		case 2:
			{
				//gnss_deviation_sub_ = new message_filters::Subscriber<autoware_msgs::GnssStandardDeviation>(nh_, gnss_deviation_topic_, 10);
				sync_velocity_ = new message_filters::Synchronizer<VelocitylocalizerSync>(VelocitylocalizerSync(SYNC_FRAMES),
								  *base_link_pose_sub_, *estimate_twist_sub_, *localizer_pose_sub_);
				sync_velocity_->registerCallback(boost::bind(&TopicList::VelocitylocalizerCallback, this, _1, _2, _3));
				break;
			}
		}
	}

	void pose_topic_publish()
	{
		//std::cout << "aaa\n" << std::flush;
		geometry_msgs::PoseStamped base_link_pose_msg;
		base_link_pose_msg.header.frame_id = "base_link";
		base_link_pose_msg.header.stamp = baselink_.header.stamp;
		base_link_pose_msg.header.seq = baselink_.header.seq;
		base_link_pose_msg.pose.position.x = baselink_.pose.position.x;
		base_link_pose_msg.pose.position.y = baselink_.pose.position.y;
		base_link_pose_msg.pose.position.z = baselink_.pose.position.z;
		base_link_pose_msg.pose.orientation.x = baselink_.pose.orientation.x;
		base_link_pose_msg.pose.orientation.y = baselink_.pose.orientation.y;
		base_link_pose_msg.pose.orientation.z = baselink_.pose.orientation.z;
		base_link_pose_msg.pose.orientation.w = baselink_.pose.orientation.w;

		tf::StampedTransform tf_stamped;
		ros::Time time = ros::Time::now();
		bool flag;std::cout << "aaa" << std::endl;
		flag = tf_listener->waitForTransform("map", twist_.header.frame_id, twist_.header.stamp, ros::Duration(3.0));
		if(flag == true) std::cout << "bbb\n" << std::flush;
		else 
		{
			return;
			std::cout << "ccc\n" << std::flush;
		}
		std::cout << twist_.header.frame_id << std::endl;
		try
		{
			tf_listener->lookupTransform("map", twist_.header.frame_id, twist_.header.stamp, tf_stamped);
		}
		catch(const tf::TransformException& e)
		{
			std::cerr << "lookup error : " << e.what() << std::endl;
			return;
		}
		//if(flag == false) std::cout << "bbb\n";
		std::cout << "ccc" << std::endl;
		tf::Quaternion qua = tf_stamped.getRotation();
		tf::Quaternion hosei = tf::createQuaternionFromYaw(yaw_correction_ * M_PI /180.0);
		tf_stamped.setRotation(qua * hosei);
		trans_broad.sendTransform(tf::StampedTransform(tf_stamped, twist_.header.stamp, "/map", "/base_link"));
		std::cout << "ddd" << std::endl;

		geometry_msgs::TwistStamped twist_pose_msg;
		twist_pose_msg.header.frame_id = "base_link";
		twist_pose_msg.header.stamp = twist_.header.stamp;
		twist_pose_msg.header.seq = twist_.header.seq;
		twist_pose_msg.twist.linear.x = twist_.twist.linear.x;
		twist_pose_msg.twist.linear.y = twist_.twist.linear.y;
		twist_pose_msg.twist.linear.z = twist_.twist.linear.z;
		twist_pose_msg.twist.angular.x = twist_.twist.angular.x;
		twist_pose_msg.twist.angular.y = twist_.twist.angular.y;
		twist_pose_msg.twist.angular.z = twist_.twist.angular.z;

		geometry_msgs::PoseStamped localizer_pose_msg;
		localizer_pose_msg.header.frame_id = "base_link";
		localizer_pose_msg.header.stamp = localizer_.header.stamp;
		localizer_pose_msg.header.seq = localizer_.header.seq;
		localizer_pose_msg.pose.position.x = localizer_.pose.position.x;
		localizer_pose_msg.pose.position.y = localizer_.pose.position.y;
		localizer_pose_msg.pose.position.z = localizer_.pose.position.z;
		localizer_pose_msg.pose.orientation.x = localizer_.pose.orientation.x;
		localizer_pose_msg.pose.orientation.y = localizer_.pose.orientation.y;
		localizer_pose_msg.pose.orientation.z = localizer_.pose.orientation.z;
		localizer_pose_msg.pose.orientation.w = localizer_.pose.orientation.w;

		//tf_Broadcaster.sendTransform(tf::StampedTransform(tf_stamped, twist_.header.stamp, "/map", "/base_link"));
		pub_current_pose_.publish(baselink_);
		pub_twist_pose_.publish(twist_pose_msg);
		pub_localizer_pose_.publish(localizer_);
	}

	void set_fusion_select(int select)
	{
		fusion_select_ = select;
	}

	void set_yaw_correction(double val)
	{
		yaw_correction_ = val;
	}
};

class LocalizerSwitch
{
private:
	ros::NodeHandle nh_, private_nh_;

	std::vector<TopicList> topic_list_;

	ros::Publisher pub_localizer_select_num_;
	ros::Subscriber sub_fusion_select_, sub_localizer_match_stat_, sub_waypoint_param_;

	autoware_config_msgs::ConfigLocalizerSwitch config_;

	bool localizer_match_flag = false;
	unsigned char localizer_select_num_;

	/*void fusion_select_callback(const std_msgs::Int32ConstPtr &msg)
	{
		localizer_select(msg->data);
	}*/

	void localizer_match_stat_callback(const autoware_msgs::LocalizerMatchStatConstPtr &msg)
	{
		if(msg->localizer_stat == true)
		{
			localizer_match_flag = true;
		}
		else
		{
			localizer_match_flag = false;
		}
		
	}

	void localizer_select(int select)
	{
		switch(select)
		{
		case 0://pose1 only
			{
				for(int i=0; i<topic_list_.size(); i++)
				{
					if(i == 0) topic_list_[0].set_fusion_select(1);
					else topic_list_[i].set_fusion_select(0);
				}
				localizer_select_num_ = 0;
				std_msgs::Int32 lsn;
				lsn.data = 0;
				pub_localizer_select_num_.publish(lsn);
				break;
			}
		case 1://pose2 only
			{
				for(int i=0; i<topic_list_.size(); i++)
				{
					if(i == 1) topic_list_[1].set_fusion_select(1);
					else topic_list_[i].set_fusion_select(0);
				}
				localizer_select_num_ = 1;
				std_msgs::Int32 lsn;
				lsn.data = 1;
				pub_localizer_select_num_.publish(lsn);
				break;
			}
		case 2://pose3 only
			{
				for(int i=0; i<topic_list_.size(); i++)
				{
					if(i == 2) topic_list_[2].set_fusion_select(1);
					else topic_list_[i].set_fusion_select(0);
				}
				localizer_select_num_ = 2;
				std_msgs::Int32 lsn;
				lsn.data = 2;
				pub_localizer_select_num_.publish(lsn);
				break;
			}
		case 10:
			{
				if(localizer_match_flag == true)
				{
					for(int i=0; i<topic_list_.size(); i++)
					{
						if(i == 0) topic_list_[0].set_fusion_select(1);
						else topic_list_[i].set_fusion_select(0);
					}
					localizer_select_num_ = 0;
					std_msgs::Int32 lsn;
					lsn.data = 10;
					pub_localizer_select_num_.publish(lsn);
				}
				break;
			}
		case 11:
			{
				if(localizer_match_flag == true)
				{
					for(int i=0; i<topic_list_.size(); i++)
					{
						if(i == 1) topic_list_[1].set_fusion_select(1);
						else topic_list_[i].set_fusion_select(0);
					}
					localizer_select_num_ = 1;
					std_msgs::Int32 lsn;
					lsn.data = 11;
					pub_localizer_select_num_.publish(lsn);
				}
				break;
			}
		case 12:
			{
				if(localizer_match_flag == true)
				{
					for(int i=0; i<topic_list_.size(); i++)
					{
						if(i == 2) topic_list_[2].set_fusion_select(1);
						else topic_list_[i].set_fusion_select(0);
					}
					localizer_select_num_ = 2;
					std_msgs::Int32 lsn;
					lsn.data = 12;
					pub_localizer_select_num_.publish(lsn);
				}
				break;
			}
		case 20:
			{
				if(localizer_select_num_ != 0)
				{
					std_msgs::Int32 lsn;
					lsn.data = -1;
					pub_localizer_select_num_.publish(lsn);
				}
				break;
			}
		case 21:
			{
				if(localizer_select_num_ != 1)
				{
					std_msgs::Int32 lsn;
					lsn.data = -1;
					pub_localizer_select_num_.publish(lsn);
				}
				break;
			}
		case 22:
			{
				if(localizer_select_num_ != 2)
				{
					std_msgs::Int32 lsn;
					lsn.data = -1;
					pub_localizer_select_num_.publish(lsn);
				}
				break;
			}
		}
	}

	void config_callback(const autoware_config_msgs::ConfigLocalizerSwitchConstPtr &msg)
	{
		std::cout << "fusuion select : " << (int)msg->fusion_select << std::endl;
		std::cout << "yaw_correction1 : " << msg->yaw_correction1 << std::endl;
		localizer_select(msg->fusion_select);

		topic_list_[0].set_yaw_correction(msg->yaw_correction1);
		topic_list_[1].set_yaw_correction(msg->yaw_correction2);
		config_ = *msg;
	}

	void waypoint_param_callback(const autoware_msgs::WaypointParamConstPtr &msg)
	{
		if(msg->ndt_yaw_correction >= -10 && msg->ndt_yaw_correction <= 10)
		{
			topic_list_[0].set_yaw_correction(msg->ndt_yaw_correction);
		}
		if(msg->gnss_yaw_correction >= -10 && msg->gnss_yaw_correction <= 10)
		{
			topic_list_[1].set_yaw_correction(msg->gnss_yaw_correction);
		}

		localizer_select(msg->fusion_select);
	}
public:
	LocalizerSwitch(ros::NodeHandle nh, ros::NodeHandle private_nh, std::vector<TopicList> list,
		int localizer_select)
	{
		nh_ = nh;  private_nh_ = private_nh;
		pub_localizer_select_num_ = nh_.advertise<std_msgs::Int32>("localizer_select_num", 1, true);

		sub_localizer_match_stat_ = nh_.subscribe<autoware_msgs::LocalizerMatchStat>(
					"/microbus/localizer_match_stat", 1, &LocalizerSwitch::localizer_match_stat_callback, this);
		sub_fusion_select_ = nh_.subscribe<autoware_config_msgs::ConfigLocalizerSwitch>(
					"/config/localizer_switch", 10, &LocalizerSwitch::config_callback, this);
		sub_waypoint_param_ = nh_.subscribe<autoware_msgs::WaypointParam>(
					"/waypoint_param", 10, &LocalizerSwitch::waypoint_param_callback, this);

		topic_list_ = list;
		for(int i=0; i<topic_list_.size(); i++)
		{
			topic_list_[i].callback_run();
		}

		/*localizer_select_num_ = localizer_select;
		std_msgs::Int32 lsn;
		lsn.data = localizer_select_num_;
		pub_localizer_select_num_.publish(lsn);*/
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ndt_matching");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	tf_listener = new tf::TransformListener;

	std::vector<TopicList> topicListArray;

	int localizer_select = 0;
	for(int cou=1; cou<=max_localizer_count; cou++)
	{
		std::string base_link_pose_topic, estimate_twist_topic, localizer_pose_topic;
		std::stringstream base_link_pose_name, estimate_twist_name, localizer_pose_name, base_link_tf_name, alignment_mechanism_name;
		std::stringstream yaw_bias_name;

		base_link_pose_name << "base_link_pose" << cou;
		private_nh.param<std::string>(base_link_pose_name.str(), base_link_pose_topic, std::string(""));
		std::cout << base_link_pose_name.str() << " : " << base_link_pose_topic << std::endl;

		estimate_twist_name << "estimate_twist" << cou;
		private_nh.param<std::string>(estimate_twist_name.str(), estimate_twist_topic, std::string(""));
		std::cout << estimate_twist_name.str() << " : " << estimate_twist_topic << std::endl;

		localizer_pose_name << "localizer_pose" << cou;
		private_nh.param<std::string>(localizer_pose_name.str(), localizer_pose_topic, std::string(""));
		std::cout << localizer_pose_name.str() << " : " << localizer_pose_topic << std::endl;

		alignment_mechanism_name << "alignment_mechanism" << cou;
		int alignment_mechanism;
		private_nh.param<int>(alignment_mechanism_name.str(), alignment_mechanism, 0);
		std::cout << alignment_mechanism_name.str() << " : " << alignment_mechanism << std::endl;

		std::string ndt_status_topic="", gnss_deviation_topic="";
		std::stringstream ndt_status_name, gnss_deviation_name;

		switch(alignment_mechanism)
		{
		case 0://ndt
			{
				ndt_status_name << "ndt_status" << cou;
				private_nh.param<std::string>(ndt_status_name.str(), ndt_status_topic, std::string(""));
				std::cout << ndt_status_name.str() << " : " << ndt_status_topic << std::endl;
				localizer_select = 0;
				break;
			}
		case 1://GNSS(RTK)
			{
				gnss_deviation_name << "gnss_deviation" << cou;
				private_nh.param<std::string>(gnss_deviation_name.str(), gnss_deviation_topic, std::string(""));
				std::cout << gnss_deviation_name.str() << " : " << gnss_deviation_topic << std::endl;
				localizer_select = 1;
				break;
			}
		case 3://velocity_localizer
			{
				localizer_select = 2;
				break;
			}
		}
	
		/*yaw_bias_name << "yaw_bias1";// << cou;
		double yaw_bias;
		private_nh.param<double>(yaw_bias_name.str(), yaw_bias, 1.0);
		std::cout << yaw_bias_name.str() << " : " << yaw_bias << std::endl;*/

		TopicList list(nh, private_nh,
					   base_link_pose_topic, estimate_twist_topic, localizer_pose_topic,
					   alignment_mechanism, ndt_status_topic, gnss_deviation_topic);
		topicListArray.push_back(list);
	}


	LocalizerSwitch localizer_switch(nh, private_nh, topicListArray, localizer_select);
	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	delete tf_listener;
	return 0;
}
