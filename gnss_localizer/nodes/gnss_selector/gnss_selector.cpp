#include <ros/ros.h>
#include <autoware_config_msgs/ConfigGnssLocalizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <autoware_msgs/GnssStandardDeviation.h>
#include <autoware_msgs/GnssSurfaceSpeed.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <autoware_system_msgs/Date.h>
#include <autoware_msgs/GnssToBaselink.h>

class GnssTopicList
{
private:
    ros::NodeHandle nh_, p_nh_;

    ros::Publisher pub_gnss_pose_, pub_gnss_surface_speed_, pub_gnss_deviation_;
    ros::Publisher pub_gnss_stat_, pub_gnss_imu_, pub_gnss_time_;

    ros::Subscriber sub_gnss_pose_, sub_gnss_surface_speed_, sub_gnss_deviation_;
    ros::Subscriber sub_gnss_stat_, sub_gnss_imu_, sub_gnss_time_;
    unsigned int publish_flag_;

    void CallbackGnssPose(const geometry_msgs::PoseStamped &msg)
    {
        if(publish_flag_) pub_gnss_pose_.publish(msg);
    }

    void CallbackGnssImu(const sensor_msgs::Imu &msg)
    {
        if(publish_flag_) pub_gnss_imu_.publish(msg);
    }

    void CallbackGnssDeviation(const autoware_msgs::GnssStandardDeviation &msg)
    {
        if(publish_flag_) pub_gnss_deviation_.publish(msg);
    }

    void CallbackGnssSurfaceSpeed(const autoware_msgs::GnssSurfaceSpeed &msg)
    {
        if(publish_flag_) pub_gnss_surface_speed_.publish(msg);
    }

    void CallbackGnssStat(const std_msgs::UInt8 &msg)
    {
        if(publish_flag_) pub_gnss_stat_.publish(msg);
    }

    void CallbackGnssTime(const autoware_system_msgs::Date &msg)
    {
        if(publish_flag_) pub_gnss_time_.publish(msg);
    }
public:
    GnssTopicList(ros::NodeHandle nh, ros::NodeHandle p_nh, std::string name_space, unsigned int class_id)
        : nh_(nh)
        , p_nh_(p_nh)
        , publish_flag_(false)
    {
        std::stringstream gnss_pose_topic;
        gnss_pose_topic << name_space << "/gnss_pose";
        sub_gnss_pose_ = nh_.subscribe(gnss_pose_topic.str(), 1, &GnssTopicList::CallbackGnssPose, this);
        pub_gnss_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/gnss_pose", 1);

        std::stringstream gnss_imu_topic;
        gnss_imu_topic << name_space << "/gnss_imu";
        sub_gnss_imu_ = nh_.subscribe(gnss_imu_topic.str(), 1, &GnssTopicList::CallbackGnssImu, this);
        pub_gnss_imu_ = nh_.advertise<sensor_msgs::Imu>("/gnss_imu", 1);

        std::stringstream gnss_deviation_topic;
        gnss_deviation_topic << name_space << "/gnss_standard_deviation";
        sub_gnss_deviation_ = nh_.subscribe(gnss_deviation_topic.str(), 1, &GnssTopicList::CallbackGnssDeviation, this);
        pub_gnss_deviation_ = nh_.advertise<autoware_msgs::GnssStandardDeviation>("/gnss_standard_deviation", 1);

        std::stringstream gnss_surface_speed_topic;
        gnss_surface_speed_topic << name_space << "/gnss_surface_speed";
        sub_gnss_surface_speed_ = nh_.subscribe(gnss_surface_speed_topic.str(), 1, &GnssTopicList::CallbackGnssSurfaceSpeed, this);
        pub_gnss_surface_speed_ = nh_.advertise<autoware_msgs::GnssSurfaceSpeed>("/gnss_surface_speed", 1);

        std::stringstream gnss_stat_topic;
        gnss_stat_topic << name_space << "/gnss_rtk_stat";
        sub_gnss_stat_ = nh_.subscribe(gnss_stat_topic.str(), 1, &GnssTopicList::CallbackGnssStat, this);
        pub_gnss_stat_ = nh_.advertise<std_msgs::UInt8>("/gnss_rtk_stat", 1);

        std::stringstream gnss_time_topic;
        gnss_time_topic << name_space << "/gnss_time";
        sub_gnss_time_ = nh_.subscribe(gnss_time_topic.str(), 1, &GnssTopicList::CallbackGnssTime, this);
        pub_gnss_time_ = nh_.advertise<autoware_system_msgs::Date>("/gnss_time", 1);
    }

    void SetPublishFlag(bool flag) {publish_flag_ = flag;}
};

class GnssSelector
{
private:
    ros::NodeHandle nh_ , p_nh_;

    ros::Publisher pub_gnss_select_, pub_gnss_to_baselink_;
    ros::Subscriber sub_config_;

    static const int TOPIC_NAME_NUM_ = 2;
    std::vector<GnssTopicList*> gnss_topic_list_;
    std::vector<double> vec_tf_gx_, vec_tf_gy_, vec_tf_gz_, vec_tf_groll_, vec_tf_gpitch_, vec_tf_gyaw_;

    void CallbackConfig(const autoware_config_msgs::ConfigGnssLocalizer &msg)
    {
        int num = atoi(msg.use_topic_num.c_str());
        for(int i=0; i<gnss_topic_list_.size(); i++)
        {
            if(num-1 == i)
            {
                autoware_msgs::GnssToBaselink gtb;
                gtb.header.stamp = msg.header.stamp;
                gtb.x = vec_tf_gx_[i];
                gtb.y = vec_tf_gy_[i];
                gtb.z = vec_tf_gz_[i];
                gtb.yaw = vec_tf_gyaw_[i];
                gtb.roll = vec_tf_groll_[i];
                gtb.pitch = vec_tf_gpitch_[i];
                pub_gnss_to_baselink_.publish(gtb);
                gnss_topic_list_[i]->SetPublishFlag(true);
            }
            else gnss_topic_list_[i]->SetPublishFlag(false);
        }
        pub_gnss_select_.publish(num);
    }
public:
    GnssSelector(ros::NodeHandle nh, ros::NodeHandle p_nh)
        : nh_(nh)
        , p_nh_(p_nh)
    {
        //std::vector<std::string> topic_list = {"gnss_pose","gnss_imu","gnss_standard_deviation","gnss_surface_speed","gnss_stat","gnss_time"}
        for(unsigned int i=0; i<TOPIC_NAME_NUM_; i++)
        {
            std::stringstream name_space;
            name_space << "namespace" << i+1;
            std::string get_name_space;
            if (p_nh_.getParam(name_space.str(), get_name_space) == false)
            {
                std::cout << get_name_space << " is not set." << std::endl;
                for(int j=0; j<i; j++) delete gnss_topic_list_[j];
                return;
            }
            std::cout << "set namespace " << get_name_space << std::endl;

            GnssTopicList *list = new GnssTopicList(nh_, p_nh_, get_name_space, i);
            gnss_topic_list_.push_back(list);

            double _tf_gx, _tf_gy, _tf_gz, _tf_groll, _tf_gpitch, _tf_gyaw;
            std::stringstream ss_gx;
            ss_gx << "gx" << i+1;
            if (nh.getParam(ss_gx.str(), _tf_gx) == false)
            {
                std::cout << ss_gx.str() << " is not set." << std::endl;
                return;
            }
            vec_tf_gx_.push_back(_tf_gx);

            std::stringstream ss_gy;
            ss_gy << "gy" << i+1;
            if (nh.getParam(ss_gy.str(), _tf_gy) == false)
            {
                std::cout << ss_gy.str() << " is not set." << std::endl;
                return;
            }
            vec_tf_gy_.push_back(_tf_gy);

            std::stringstream ss_gz;
            ss_gz << "gz" << i+1;
            if (nh.getParam(ss_gz.str(), _tf_gz) == false)
            {
                std::cout << ss_gz.str() << " is not set." << std::endl;
                return;
            }
            vec_tf_gz_.push_back(_tf_gz);

            std::stringstream ss_yaw;
            ss_yaw << "gyaw" << i+1;
            if (nh.getParam(ss_yaw.str(), _tf_gyaw) == false)
            {
                std::cout << ss_yaw.str() << " is not set." << std::endl;
                return;
            }
            vec_tf_gyaw_.push_back(_tf_gyaw);

            std::stringstream ss_roll;
            ss_roll << "groll" << i+1;
            if (nh.getParam(ss_roll.str(), _tf_groll) == false)
            {
                std::cout << ss_roll.str() << " is not set." << std::endl;
                return;
            }
            vec_tf_groll_.push_back(_tf_groll);

            std::stringstream ss_pitch;
            ss_pitch << "gpitch" << i+1;
            if (nh.getParam(ss_pitch.str(), _tf_gpitch) == false)
            {
                std::cout << ss_pitch.str() << " is not set." << std::endl;
                return;
            }
            vec_tf_gpitch_.push_back(_tf_gpitch);
        }

        pub_gnss_select_ = nh_.advertise<std_msgs::Int32>("/gnss_select", 1, true);
        pub_gnss_to_baselink_ = nh_.advertise<autoware_msgs::GnssToBaselink>("/gnss_to_base_link", 1, true);
        sub_config_ = nh.subscribe("/config/gnss_localizer", 1, &GnssSelector::CallbackConfig, this);std::cout << "aaa" << std::endl;
    }

    ~GnssSelector()
    {
        for(int i=0; i<gnss_topic_list_.size(); i++) delete gnss_topic_list_[i];
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnss_selector");
    //pthread_mutex_init(&mutex, NULL);

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    GnssSelector selector(nh, private_nh);

    ros::spin();
    return 0;
}