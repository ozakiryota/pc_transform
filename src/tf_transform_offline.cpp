#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

class PcTfTransformOffline{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*publisher*/
		ros::Publisher tf_pub_;
		ros::Publisher tf_static_pub_;
        std::vector<ros::Publisher> pc_pub_list_;
        /*buffer*/
        tf::TransformListener tf_listener_;
		/*parameter*/
		std::string load_rosbag_path_;
		std::string save_rosbag_path_;
		std::string target_frame_;
		std::string save_topic_child_name_;
        float debug_hz_;
        std::vector<std::string> topic_name_list_;
        /*function*/
        void republishTF(const rosbag::View::iterator& view_itr);
        void transformPC(sensor_msgs::PointCloud2& pc2);
        void writePC(const sensor_msgs::PointCloud2& pc2, const std::string& save_topic_name);
        void publishDebugPC(const sensor_msgs::PointCloud2& pc2, const std::string& topic_name);

	public:
		PcTfTransformOffline();
        void load();
};

PcTfTransformOffline::PcTfTransformOffline()
	: nh_private_("~")
{
	std::cout << "----- pc_tf_transform_offline -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("load_rosbag_path", load_rosbag_path_)){
        std::cerr << "Set load_rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "load_rosbag_path_ = " << load_rosbag_path_ << std::endl;
    nh_private_.param("save_rosbag_path", save_rosbag_path_, std::string(load_rosbag_path_.substr(0, load_rosbag_path_.length() - 4) + "_transformed.bag"));
	std::cout << "save_rosbag_path_ = " << save_rosbag_path_ << std::endl;

    if(!nh_private_.getParam("target_frame", target_frame_)){
        std::cerr << "Set target_frame." << std::endl; 
        exit(true);
    }
	std::cout << "target_frame_ = " << target_frame_ << std::endl;
    nh_private_.param("save_topic_child_name", save_topic_child_name_, std::string("transformed"));
	std::cout << "save_topic_child_name_ = " << save_topic_child_name_ << std::endl;

    nh_private_.param("debug_hz", debug_hz_, float(-1));
	std::cout << "debug_hz_ = " << debug_hz_ << std::endl;

    for(size_t i = 0; ; i++){
        std::string tmp_topic_name;
        if(!nh_private_.getParam("topic_" + std::to_string(i), tmp_topic_name))  break;
        topic_name_list_.push_back(tmp_topic_name);
        std::cout << "topic_name_list_[" << i << "] = " << topic_name_list_[i] << std::endl;
    }

    /*publisher*/
	tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 1);
	tf_static_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf_static", 1);
    for(const std::string& topic_name : topic_name_list_){
        pc_pub_list_.push_back(nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1));
        pc_pub_list_.push_back(nh_.advertise<sensor_msgs::PointCloud2>(topic_name + "/" + save_topic_child_name_, 1));
    }

    /*file*/
    std::filesystem::copy(load_rosbag_path_, save_rosbag_path_, std::filesystem::copy_options::overwrite_existing);
}

void PcTfTransformOffline::load()
{
    rosbag::Bag bag;
    try{
        bag.open(load_rosbag_path_, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << load_rosbag_path_ << std::endl;
        exit(true);
    }

    rosbag::View view(bag, rosbag::TopicQuery(topic_name_list_));
    rosbag::View::iterator view_itr;
    view.addQuery(bag, rosbag::TypeQuery("tf2_msgs/TFMessage"));
    view_itr = view.begin();

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        if(view_itr->getDataType() == "tf2_msgs/TFMessage") republishTF(view_itr);
        else if(view_itr->getDataType() == "sensor_msgs/PointCloud2"){
            for(size_t i = 0; i < topic_name_list_.size(); i++){
                if(view_itr->getTopic() == topic_name_list_[i]){
                    sensor_msgs::PointCloud2Ptr pc_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
                    publishDebugPC(*pc_ptr, view_itr->getTopic());
                    transformPC(*pc_ptr);
                    writePC(*pc_ptr, view_itr->getTopic() + "/" + save_topic_child_name_);
                    publishDebugPC(*pc_ptr, view_itr->getTopic() + "/" + save_topic_child_name_);
                    break;
                }
            }
        }
        if(debug_hz_ > 0)    loop_rate.sleep();
        view_itr++;
    }

    bag.close();
}

void PcTfTransformOffline::republishTF(const rosbag::View::iterator& view_itr)
{
    if(view_itr->getTopic() == "/tf")   tf_pub_.publish(view_itr->instantiate<tf2_msgs::TFMessage>());
    if(view_itr->getTopic() == "/tf_static")   tf_static_pub_.publish(view_itr->instantiate<tf2_msgs::TFMessage>());
}

void PcTfTransformOffline::transformPC(sensor_msgs::PointCloud2& pc2)
{
    sensor_msgs::PointCloud pc1;
	sensor_msgs::convertPointCloud2ToPointCloud(pc2, pc1);
	try{
		tf_listener_.waitForTransform(target_frame_, pc2.header.frame_id, ros::Time(0), ros::Duration(0.1));
		tf_listener_.transformPointCloud(target_frame_, ros::Time(0), pc1, pc2.header.frame_id, pc1);
        pc1.header.frame_id = target_frame_;
        pc1.header.stamp = pc2.header.stamp;
		sensor_msgs::convertPointCloudToPointCloud2(pc1, pc2);
	}
	catch(const tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
	}
}

void PcTfTransformOffline::writePC(const sensor_msgs::PointCloud2& pc2, const std::string& save_topic_name)
{
    rosbag::Bag bag;
    try{
        bag.open(save_rosbag_path_, rosbag::bagmode::Append);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << save_rosbag_path_ << std::endl;
        exit(true);
    }
    bag.write(save_topic_name, pc2.header.stamp, pc2);
    bag.close();
}

void PcTfTransformOffline::publishDebugPC(const sensor_msgs::PointCloud2& pc2, const std::string& topic_name)
{
    sensor_msgs::PointCloud2 debug_pc = pc2;
    debug_pc.header.stamp = ros::Time::now();

    for(const ros::Publisher& pub : pc_pub_list_){
        if(pub.getTopic() == topic_name){
            pub.publish(debug_pc);
            break;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_tf_transform_offline");
	
	PcTfTransformOffline pc_tf_transform_offline;
    pc_tf_transform_offline.load();
}