#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

class PCTFTransform{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pc;
		/*publisher*/
		ros::Publisher _pub_pc;
		/*tf*/
		tf::TransformListener _tflistener;
		/*parameters*/
		std::string _target_frame;

	public:
		PCTFTransform();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void transformPC(const sensor_msgs::PointCloud2& pc2_in);
		void publication(void);
};

PCTFTransform::PCTFTransform()
	: _nhPrivate("~")
{
	std::cout << "--- pc_tf_transform ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("target_frame", _target_frame, std::string("/target_frame"));
	std::cout << "_target_frame = " << _target_frame << std::endl;
	/*subscriber*/
	_sub_pc = _nh.subscribe("/cloud", 1, &PCTFTransform::callbackPC, this);
	/*publisher*/
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/transformed", 1);
}

void PCTFTransform::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	transformPC(*msg);
}

void PCTFTransform::transformPC(const sensor_msgs::PointCloud2& pc2_in)
{
	sensor_msgs::PointCloud pc1_in;
	sensor_msgs::PointCloud pc1_trans;
	sensor_msgs::PointCloud2 pc2_trans;

	sensor_msgs::convertPointCloud2ToPointCloud(pc2_in, pc1_in);
	try{
		_tflistener.waitForTransform(_target_frame, pc2_in.header.frame_id, pc2_in.header.stamp, ros::Duration(1.0));
		_tflistener.transformPointCloud(_target_frame, pc2_in.header.stamp, pc1_in, pc2_in.header.frame_id, pc1_trans);
		sensor_msgs::convertPointCloudToPointCloud2(pc1_trans, pc2_trans);
		_pub_pc.publish(pc2_trans);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_tf_transform");
	
	PCTFTransform pc_tf_transform;

	ros::spin();
}
