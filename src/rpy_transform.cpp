#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

class PcRpyTransform{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_;
		/*publisher*/
		ros::Publisher pub_;
		/*parameter*/
		std::string publish_frame_;
		double x_m_, y_m_, z_m_;
		double r_deg_, p_deg_, y_deg_;

	public:
		PcRpyTransform();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void transformPC(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		void publication(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		double degToRad(double deg);
};

PcRpyTransform::PcRpyTransform()
	: nh_private_("~")
{
	std::cout << "--- pc_rpy_transform ---" << std::endl;
	/*parameter*/
	nh_private_.param("publish_frame", publish_frame_, std::string(""));
	std::cout << "publish_frame_ = " << publish_frame_ << std::endl;
	nh_private_.param("x_m", x_m_, 0.0);
	std::cout << "x_m_ = " << x_m_ << std::endl;
	nh_private_.param("y_m", y_m_, 0.0);
	std::cout << "y_m_ = " << y_m_ << std::endl;
	nh_private_.param("z_m", z_m_, 0.0);
	std::cout << "z_m_ = " << z_m_ << std::endl;
	nh_private_.param("r_deg", r_deg_, 0.0);
	std::cout << "r_deg_ = " << r_deg_ << std::endl;
	nh_private_.param("p_deg", p_deg_, 0.0);
	std::cout << "p_deg_ = " << p_deg_ << std::endl;
	nh_private_.param("y_deg", y_deg_, 0.0);
	std::cout << "y_deg_ = " << y_deg_ << std::endl;
	/*subscriber*/
	sub_ = nh_.subscribe("/point_cloud", 1, &PcRpyTransform::callbackPC, this);
	/*publisher*/
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/transformed", 1);
}

void PcRpyTransform::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pc);
	transformPC(pc);
	publication(pc);
}

void PcRpyTransform::transformPC(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
	Eigen::Affine3f transformatoin = pcl::getTransformation(x_m_, y_m_, z_m_, degToRad(r_deg_), degToRad(p_deg_), degToRad(y_deg_));
	pcl::transformPointCloud(*pc, *pc, transformatoin);
}

void PcRpyTransform::publication(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
    sensor_msgs::PointCloud2 ros_pc;
    pcl::toROSMsg(*pc, ros_pc);
	if(publish_frame_ != "")	ros_pc.header.frame_id = publish_frame_;
    pub_.publish(ros_pc);
}

double PcRpyTransform::degToRad(double deg)
{
	double rad = deg / 180.0 * M_PI;
	rad = atan2(sin(rad), cos(rad));
	return rad;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_rpy_transform");
	
	PcRpyTransform pc_rpy_transform;

	ros::spin();
}
