#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

class PcTransform{
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
		PcTransform();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void transformPC(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		void publication(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		double degToRad(double deg);
};

PcTransform::PcTransform()
	: nh_private_("~")
{
	std::cout << "--- pc_transform ---" << std::endl;
	/*parameter*/
	nh_private_.param("target_frame", publish_frame_, std::string("target_frame"));
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
	sub_ = nh_.subscribe("/point_cloud", 1, &PcTransform::callbackPC, this);
	/*publisher*/
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/transformed", 1);
}

void PcTransform::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pc);
	transformPC(pc);
	publication(pc);
}

void PcTransform::transformPC(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
	Eigen::Quaternionf rotation =
		Eigen::AngleAxisf(degToRad(r_deg_), Eigen::Vector3f::UnitX())
    	* Eigen::AngleAxisf(degToRad(p_deg_), Eigen::Vector3f::UnitY())
    	* Eigen::AngleAxisf(degToRad(y_deg_), Eigen::Vector3f::UnitZ());
	Eigen::Vector3f offset(x_m_, y_m_, z_m_);

	pcl::transformPointCloud(*pc, *pc, offset, rotation);
}

void PcTransform::publication(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
    sensor_msgs::PointCloud2 ros_pc;
    pcl::toROSMsg(*pc, ros_pc);
    pub_.publish(ros_pc);
}

double PcTransform::degToRad(double deg)
{
	double rad = deg / 180.0 * M_PI;
	rad = atan2(sin(rad), cos(rad));
	return rad;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_transform");
	
	PcTransform pc_transform;

	ros::spin();
}
