#include <cmath>
#include <memory>
#include <chrono>
#include <stdio.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.h>
#include "livox_interfaces/msg/custom_msg.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/transform_datatypes.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;

typedef pcl::PointXYZI PointType;

// Global variables
//=================================================================================================
std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_datas;
std::vector<nav_msgs::msg::Odometry> imu_datas;
std::vector<sensor_msgs::msg::NavSatFix> rtk_datas;

double lidar_delta_time = 0.01; //100Hz lidar data, you can change this parameter by your sensor

const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
Eigen::Matrix4d rtk2lidar; //the extrinsic parameter

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_apx_p2;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_apx_rpy;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());

rclcpp::TimerBase::SharedPtr timer;
rclcpp::Time last_msg_time;

// Prototypes
//=================================================================================================
bool lidar_imu_rtk_process(uint32_t num_lidar, uint32_t num_imu, uint32_t num_rtk);

bool average_quaternion(nav_msgs::msg::Odometry &start, nav_msgs::msg::Odometry &end, Eigen::Quaterniond &result, double t);

bool mercator_proj(double B0, double L0, double B, double L, double &X, double&Y);

void RGBTrans(PointType const * const pi, pcl::PointXYZRGB * const po);

void savePointCloudToFile(const std::string& map_file_path,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& laserCloudFullResColor_pcd,
                          const rclcpp::Logger &logger);


// Time functions
//=================================================================================================
inline double to_time(sensor_msgs::msg::PointCloud2::SharedPtr lidar_data) {
    return (lidar_data->header.stamp.sec )*1.0 + (lidar_data->header.stamp.nanosec / 1000000000.0);
}
inline double to_time(nav_msgs::msg::Odometry imu_data) {
    return (imu_data.header.stamp.sec )*1.0 + (imu_data.header.stamp.nanosec / 1000000000.0);
}
inline double to_time(sensor_msgs::msg::NavSatFix rtk_data) {
    return (rtk_data.header.stamp.sec )*1.0 + (rtk_data.header.stamp.nanosec / 1000000000.0);
}

inline double to_time(livox_interfaces::msg::CustomMsg lidar_data) {
    // return (lidar_data.timebase / 1000000 % 10000000) / 1000.0;
    return lidar_data.timebase / 1000000000.0;
}

// Callbacks
//=================================================================================================
void apxCbk(const livox_interfaces::msg::CustomMsg::SharedPtr msg)
{

    pcl::PointCloud<PointType> laserCloud;

    pcl::PointCloud<PointType>::Ptr laserCloud2(new pcl::PointCloud<PointType>());

    laserCloud2->is_dense = false;
    laserCloud2->height = 1;
    laserCloud2->width = msg->point_num;
    laserCloud2->points.resize(msg->point_num);

    //std::cout<<"DEBUG apxCbk timebase "<< msg->timebase <<std::endl;

    for(int i = 0; i < msg->point_num; i++){
        laserCloud2->points[i].x = msg->points[i].x;
        laserCloud2->points[i].y = msg->points[i].y;
        laserCloud2->points[i].z = msg->points[i].z;
        laserCloud2->points[i].intensity = msg->points[i].reflectivity;
    }

    sensor_msgs::msg::PointCloud2 p2_apx;

    pcl::toROSMsg(*laserCloud2, p2_apx);
    p2_apx.header.frame_id = "/camera_init";
    p2_apx.header.stamp = msg->header.stamp;
    pub_apx_p2->publish(p2_apx);
}

void lidarCbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "WTF???");
    last_msg_time = node->now();
    if(lidar_datas.size() > 0)
    {
        if(to_time(lidar_datas[lidar_datas.size()-1]) > to_time(msg))
        {
            RCLCPP_INFO(node->get_logger(), "lidar time error");
            return;
        }
    }
    lidar_datas.push_back(msg);
}

void imuCbk(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    imu_datas.push_back(*msg);
}
void apximuCbk(const sensor_msgs::msg::Imu::SharedPtr msg) // transform sensor_msgs::Imu to nav_msgs::Odometry
{
    nav_msgs::msg::Odometry tempOdo;
    tempOdo.header.stamp = msg->header.stamp;
    tempOdo.pose.pose.orientation.x = msg->orientation.x;
    tempOdo.pose.pose.orientation.y = msg->orientation.y;
    tempOdo.pose.pose.orientation.z = msg->orientation.z;
    tempOdo.pose.pose.orientation.w = msg->orientation.w;
    tempOdo.header.frame_id = "/camera_init";

    pub_apx_rpy->publish(tempOdo);
}
void rtkCbk(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    rtk_datas.push_back(*msg);
}

// Main
//=================================================================================================
int main(int argc, char *argv[]) {
    rtk2lidar << -1,  0, 0, 0,
                  0, -1, 0, 0,
                  0,  0, 1, 0,
                  0,  0, 0, 1;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("livox_mapping_case");

    auto logger = node->get_logger();

    auto sub_rtk = node->create_subscription<sensor_msgs::msg::NavSatFix> ("/fix", 1000, rtkCbk);
    auto sub_apx_rpy = node->create_subscription<sensor_msgs::msg::Imu> ("/livox/imu", 20000, apximuCbk);
    pub_apx_rpy = node->create_publisher<nav_msgs::msg::Odometry> ("pub_apx_rpy", 1000);

    auto sub_point = node->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/livox/lidar", 1000, [node](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                    lidarCbk(msg, node);
                                });
    auto sub_imu = node->create_subscription<nav_msgs::msg::Odometry> ("pub_apx_rpy", 20000, imuCbk);

    pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2> ("pub_pointcloud2", 1);
    pub_odometry = node->create_publisher<nav_msgs::msg::Odometry> ("pub_odometry", 1);

    std::string map_file_path;
    node->declare_parameter<std::string> ("map_file_path", "");
    node->get_parameter("map_file_path", map_file_path);

    uint32_t num_lidar = 0;
    uint32_t num_imu = 1;
    uint32_t num_rtk = 1;

    uint32_t num_lidar_last = 0;
    uint32_t num_imu_last = 1;
    uint32_t num_rtk_last = 1;

    bool init_flag = false;

    bool stop_flag = false;

    timer = node->create_wall_timer(
        1s,
        [&]() {
            auto time_sinse_last_msg = (node->now() - last_msg_time).seconds();
            if (time_sinse_last_msg > 5.0) {
                RCLCPP_INFO(logger, "No more data, shutting down...");
                stop_flag = true;
            }
        }
    );

    last_msg_time = node->now();
    while (rclcpp::ok() && !stop_flag) {
        rclcpp::spin_some(node);

        if (num_lidar < lidar_datas.size()) {
            bool imu_flag = false;
            bool rtk_flag = false;

            // imu data align
            if (num_imu < imu_datas.size()) {
                if (to_time(imu_datas[num_imu - 1]) <= to_time(lidar_datas[num_lidar])) {
                    if (to_time(imu_datas[num_imu]) >= to_time(lidar_datas[num_lidar])) {
                        imu_flag = true;
                    } else {
                        num_imu++;
                    }
                } else {
                    num_lidar++;
                    continue;
                }
            }

            // rtk data align
            if (num_rtk < rtk_datas.size()) {
                if (to_time(rtk_datas[num_rtk - 1]) <= to_time(lidar_datas[num_lidar])) {
                    if (to_time(rtk_datas[num_rtk]) >= to_time(lidar_datas[num_lidar])) {
                        rtk_flag = true;
                    } else {
                        num_rtk++;
                    }
                } else {
                    num_lidar++;
                    continue;
                }
            }

            if (imu_flag & rtk_flag) {
                if (init_flag) {
                    if (!lidar_imu_rtk_process(num_lidar_last, num_imu_last-1, num_rtk_last)) {
                        std::cout << "error happened" << std::endl; // TODO more verbosity output pls
                        return -1;
                    }
                    num_lidar_last = num_lidar;
                    num_imu_last = num_imu;
                    num_rtk_last = num_rtk;
                } else {
                    init_flag = true;
                    num_lidar_last = num_lidar;
                    num_imu_last = num_imu;
                    num_rtk_last = num_rtk;
                }
                num_lidar++;
            }
        }
    }

    RCLCPP_INFO(logger, "%s", map_file_path.c_str());

    if (!map_file_path.empty()) {
        savePointCloudToFile(map_file_path, laserCloudFullResColor_pcd, logger);
    }

    pub_cloud.reset();
    pub_odometry.reset();
    rclcpp::shutdown();
    return 0;
}


bool lidar_imu_rtk_process(uint32_t num_lidar, uint32_t num_imu, uint32_t num_rtk) {
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    pcl::fromROSMsg(*lidar_datas[num_lidar], laserCloudIn);

    double lidar_t = to_time(lidar_datas[num_lidar]);
    double imu_back_t = to_time(imu_datas[num_imu+1]);
    double rtk_back_t = to_time(rtk_datas[num_rtk]);

    double dt = lidar_delta_time / laserCloudIn.points.size();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());

    Eigen::Matrix3d rot;
    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

    // with Distortion correction
    for (int i = 0; i < laserCloudIn.points.size(); i++) {
        // if over time use the next
        if (lidar_t > imu_back_t) {
            num_imu++;
            imu_back_t = to_time(imu_datas[num_imu+1]);
        }

        if (lidar_t > rtk_back_t) {
            num_rtk++;
            rtk_back_t = to_time(rtk_datas[num_rtk]);
        }

        double temp_t = lidar_t - to_time(imu_datas[num_imu]);
        double t22 = to_time(imu_datas[num_imu+1]) - to_time(imu_datas[num_imu]);

        Eigen::Quaterniond q;

        if (!average_quaternion(imu_datas[num_imu], imu_datas[num_imu+1], q, temp_t/t22)) {
            continue;
        }
        rot = q.normalized().toRotationMatrix();

        double LLA[3], p[3];
        double t1 = lidar_t - to_time(rtk_datas[num_rtk-1]);
        double t2 = to_time(rtk_datas[num_rtk]) - to_time(rtk_datas[num_rtk-1]);
        LLA[0] = rtk_datas[num_rtk-1].longitude*(1-t1/t2) + rtk_datas[num_rtk].longitude*(t1/t2);
        LLA[1] = rtk_datas[num_rtk-1].latitude*(1-t1/t2) + rtk_datas[num_rtk].latitude*(t1/t2);
        LLA[2] = rtk_datas[num_rtk-1].altitude*(1-t1/t2) + rtk_datas[num_rtk].altitude*(t1/t2);

        static const double LLA0[3] = {LLA[0], LLA[1], LLA[2]};

        if(!mercator_proj(LLA0[1]*M_PI/180, LLA0[0]*M_PI/180, LLA[1]*M_PI/180, LLA[0]*M_PI/180, p[0], p[1]))
        {
            // Mercator projection
            printf("No mercator\n");
            continue;
        }
        p[2] = LLA[2];

        static const double p0[3] = {p[0], p[1], p[2]};

        p[0] = p[0] - p0[0];
        p[1] = p[1] - p0[1];
        p[2] = p0[2] - p[2];

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        Eigen::Vector3d cn;
        cn << p[0], p[1], p[2];

        T.block<3, 3>(0, 0) = rot;
        T.block<3, 1>(0, 3) =  cn;

        static const Eigen::Matrix4d T1 = T;

        trans = rtk2lidar * T1.inverse() * T * rtk2lidar.inverse();

        Eigen::Matrix<double,4,1> or_point,after_point;
        or_point << laserCloudIn.points[i].x, laserCloudIn.points[i].y, laserCloudIn.points[i].z, 1;

        //Eigen::Matrix4d trans_inv = trans.inverse();

        after_point = trans * or_point;

        pcl::PointXYZI temp_after_point;
        temp_after_point.x = after_point[0];
        temp_after_point.y = after_point[1];
        temp_after_point.z = after_point[2];
        temp_after_point.intensity = laserCloudIn.points[i].intensity;

        //temp_after_point.z = -temp_after_point.z;

        pcl::PointXYZRGB temp_point;
        RGBTrans(&temp_after_point, &temp_point);
        laserCloudFullResColor->push_back(temp_point);

        lidar_t += dt;
    }

    Eigen::Matrix3d rotation_R = trans.block<3,3>(0,0);
    Eigen::Quaterniond QQ(rotation_R);

    nav_msgs::msg::Odometry odo_output;
    odo_output.header.frame_id = "/camera_init";
    odo_output.child_frame_id = "/livox";
    odo_output.pose.pose.orientation.w = QQ.w();
    odo_output.pose.pose.orientation.x = QQ.x();
    odo_output.pose.pose.orientation.y = QQ.y();
    odo_output.pose.pose.orientation.z = QQ.z();
    odo_output.pose.pose.position.x = trans(0, 3);
    odo_output.pose.pose.position.y = trans(1, 3);
    odo_output.pose.pose.position.z = trans(2, 3);
    pub_odometry->publish(odo_output);

    auto tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>
        (*rclcpp::Node::make_shared("tf_broadcaster_node"));
    geometry_msgs::msg::TransformStamped aftMappedTrans;

    aftMappedTrans.header.stamp = rclcpp::Clock().now();
    aftMappedTrans.header.frame_id = "/camera_init";
    aftMappedTrans.child_frame_id = "/aft_mapped";

    tf2::Quaternion tfQ;
    tfQ.setW(QQ.w());
    tfQ.setX(QQ.x());;
    tfQ.setY(QQ.y());
    tfQ.setZ(QQ.z());

    aftMappedTrans.transform.rotation.x = tfQ.x();
    aftMappedTrans.transform.rotation.y = tfQ.y();
    aftMappedTrans.transform.rotation.z = tfQ.z();
    aftMappedTrans.transform.rotation.w = tfQ.w();
    aftMappedTrans.transform.translation.x = trans(0, 3);
    aftMappedTrans.transform.translation.y = trans(1, 3);
    aftMappedTrans.transform.translation.z = trans(2, 3);

    tfBroadcaster->sendTransform(aftMappedTrans);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*laserCloudFullResColor, output);
    output.header.frame_id = "/camera_init";
    pub_cloud->publish(output);

    *laserCloudFullResColor_pcd += *laserCloudFullResColor;

    return true;
}

void RGBTrans(PointType const * const pi, pcl::PointXYZRGB * const po) {
    po->x = pi->x;
    po->y = pi->y;
    po->z = pi->z;

    int reflection_map = pi->intensity;

    if (reflection_map < 30)
    {
        int green = (reflection_map * 255 / 30);
        po->r = 0;
        po->g = green & 0xff;
        po->b = 0xff;
    }
    else if (reflection_map < 90)
    {
        int blue = (((90 - reflection_map) * 255) / 60);
        po->r = 0x0;
        po->g = 0xff;
        po->b = blue & 0xff;
    }
    else if (reflection_map < 150)
    {
        int red = ((reflection_map-90) * 255 / 60);
        po->r = red & 0xff;
        po->g = 0xff;
        po->b = 0x0;
    }
    else
    {
        int green = (((255-reflection_map) * 255) / (255-150));
        po->r = 0xff;
        po->g = green & 0xff;
        po->b = 0;
    }
}

//Quaternion interpolation
bool average_quaternion(nav_msgs::msg::Odometry &start, nav_msgs::msg::Odometry &end, Eigen::Quaterniond &result, double t) {
    if((t > 1.0) || (t < 0.0))
    {
        return false;
    }
    double starting[4] = {start.pose.pose.orientation.w, start.pose.pose.orientation.x, start.pose.pose.orientation.y, start.pose.pose.orientation.z};
    double ending[4] = {end.pose.pose.orientation.w, end.pose.pose.orientation.x, end.pose.pose.orientation.y, end.pose.pose.orientation.z};
    double cosa = starting[0]*ending[0] + starting[1]*ending[1] + starting[2]*ending[2] + starting[3]*ending[3];
    if(cosa < 0.0)
    {
        ending[0] = -ending[0];
        ending[1] = -ending[1];
        ending[2] = -ending[2];
        ending[3] = -ending[3];
        cosa = -cosa;
    }

    double k0, k1;
    if(cosa > 0.9995)
    {
        k0 = 1.0 - t;
        k1 = t;
    }
    else
    {
        double sina = sqrt(1.0 - cosa * cosa);
        double a = atan2(sina, cosa);
        k0 = sin((1.0 - t) * a) / sina;
        k1 = sin(t * a) / sina;
    }

    result.w() = starting[0]*k0 + ending[0]*k1;
    result.x() = starting[1]*k0 + ending[1]*k1;
    result.y() = starting[2]*k0 + ending[2]*k1;
    result.z() = starting[3]*k0 + ending[3]*k1;
    return true;
}

bool mercator_proj(double B0, double L0, double B, double L, double &X, double&Y) {
    static double _A = 6378137, _B = 6356752.3142, _B0 = B0, _L0 = L0;//_B0 = 22 * M_PI / 180, _L0 = 0;
    static double e = sqrt(1 - (_B/_A)*(_B/_A));
    static double e_ = sqrt((_A/_B)*(_A/_B) - 1);
    static double NB0 = ((_A*_A)/_B) / sqrt(1+e_*e_*cos(_B0)*cos(_B0));
    static double K = NB0 * cos(_B0);
    static double E = exp(1);

    if((L < -M_PI) || (L > M_PI) || (B < -M_PI_2) || (B > M_PI_2))
    {
        return false;
    }

    Y = K * (L - _L0);
    X = K * log(tan(M_PI_4+B/2) * pow((1-e*sin(B))/(1+e*sin(B)), e/2));

    return true;
}

void savePointCloudToFile(const std::string& map_file_path,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& laserCloudFullResColor_pcd,
    const rclcpp::Logger &logger)
{
    std::string all_points_filename = map_file_path + "/all_points.pcd";

    try {
        if (pcl::io::savePCDFileBinary(all_points_filename, *laserCloudFullResColor_pcd) == -1)
        {
            RCLCPP_ERROR(logger, "Failed to save point cloud to %s", all_points_filename.c_str());
            return;
        }
        RCLCPP_INFO(logger, "Point cloud saved successfully to %s", all_points_filename.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Exception while saving point cloud: %s", e.what());
    }
}