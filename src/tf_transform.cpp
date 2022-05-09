#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

class PcTfTransform{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_;
		/*publisher*/
		ros::Publisher pub_;
		/*tf*/
		tf::TransformListener tf_listener_;
		/*parameter*/
		std::string target_frame_;
		bool use_msg_stamp_;

	public:
		PcTfTransform();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void transformPC(const sensor_msgs::PointCloud2& pc2_in);
		void publication(void);
};

PcTfTransform::PcTfTransform()
	: nh_private_("~")
{
	std::cout << "--- tf_transform ---" << std::endl;
	/*parameter*/
	nh_private_.param("target_frame", target_frame_, std::string("target_frame"));
	std::cout << "target_frame_ = " << target_frame_ << std::endl;
	nh_private_.param("use_msg_stamp", use_msg_stamp_, true);
	std::cout << "use_msg_stamp_ = " << (bool)use_msg_stamp_ << std::endl;
	/*subscriber*/
	sub_ = nh_.subscribe("/point_cloud", 1, &PcTfTransform::callbackPC, this);
	/*publisher*/
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/transformed", 1);
}

void PcTfTransform::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	transformPC(*msg);
}

void PcTfTransform::transformPC(const sensor_msgs::PointCloud2& pc2_in)
{
	sensor_msgs::PointCloud pc1_in;
	sensor_msgs::PointCloud pc1_trans;
	sensor_msgs::PointCloud2 pc2_trans;

	sensor_msgs::convertPointCloud2ToPointCloud(pc2_in, pc1_in);
	try{
		if(use_msg_stamp_){
			tf_listener_.waitForTransform(target_frame_, pc2_in.header.frame_id, pc2_in.header.stamp, ros::Duration(1.0));
			tf_listener_.transformPointCloud(target_frame_, pc2_in.header.stamp, pc1_in, pc2_in.header.frame_id, pc1_trans);
		}
		else{
			tf_listener_.waitForTransform(target_frame_, pc2_in.header.frame_id, ros::Time(0), ros::Duration(1.0));
			tf_listener_.transformPointCloud(target_frame_, ros::Time(0), pc1_in, pc2_in.header.frame_id, pc1_trans);
		}
		sensor_msgs::convertPointCloudToPointCloud2(pc1_trans, pc2_trans);
		pub_.publish(pc2_trans);
	}
	catch(const tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_transform");
	
	PcTfTransform tf_transform;

	ros::spin();
}
