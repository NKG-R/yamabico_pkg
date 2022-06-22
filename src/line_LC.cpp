#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

nav_msgs::Odometry pos;
sensor_msgs::LaserScan scan;
geometry_msgs::Twist pub_msg;
double start_x = 0.0;   // 移動開始時のx
double start_y = 0.0;   // 移動開始時のy
double start_yaw = 0.0; // 移動開始時の角度

int state = 0;
double scan_coord[726][2]; // scanデータの座標
double line_x = 0.0;       // 追従直線が通る点のx
double line_y = 0.0;       // 追従直線が通る点のy
double line_theta = 0.0;   // 追従直線の角度

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos.header = msg->header;
    pos.child_frame_id = msg->child_frame_id;
    pos.pose = msg->pose;
    pos.twist = msg->twist;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
    scan.header = scan_->header;
    scan.angle_min = scan_->angle_min;
    scan.angle_max = scan_->angle_max;
    scan.angle_increment = scan_->angle_increment;
    scan.time_increment = scan_->time_increment;
    scan.scan_time = scan_->scan_time;
    scan.range_min = scan_->range_min;
    scan.range_max = scan_->range_max;
    scan.ranges = scan_->ranges;
    scan.intensities = scan_->intensities;
}

// ロボットの角度取得
double get_yaw()
{
    tf::Quaternion quat(pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// scanデータを2次元座標に変換
void scan2coord()
{
    int i = 0;
    while (i < scan.ranges.size())
    {
        double th = i * 255.0 / 725.0;
        th = th - 135.0;
        // th = th - 120.0;
        double theta = th * M_PI / 180.0;
        if (!(scan.ranges[i] <= 5.0 && scan.ranges[i] >= 0.1))
            scan.ranges[i] = 0.0;
        scan_coord[i][0] = scan.ranges[i] * cos(theta);
        scan_coord[i][1] = scan.ranges[i] * sin(theta);
        i++;
    }
}

//　指定した座標付近で停止
int near_position(double goal_x, double goal_y)
{
    double difx = (pos.pose.pose.position.x - start_x) - goal_x;
    double dify = (pos.pose.pose.position.y - start_y) - goal_y;
    return (sqrt(difx * difx + dify * dify) < 0.2);
}

// 壁から0.8mの直線を計算
void wall2line() {}

// 直線追従
void line_LC(double x, double y, double th)
{
    // 制御のパラメータ(調整必須)
    const double k_eta = 350.0;
    const double k_phai = 200.0;
    const double k_w = 200.0;

    // 角速度の最大値
    const double w_max = 0.2;

    // 壁から離す距離
    const double distance = 0.8;

    // 現在の角速度
    double w0 = pos.twist.twist.angular.z;

    // ロボットと直線の距離
    double eta = 0;
    if (th == M_PI / 2.0)
        eta = x;
    else if (th == -M_PI / 2.0)
        eta = -x;
    else if (abs(th) < M_PI / 2.0)
        eta = (- y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1) - distance;
    else
        eta = -((- y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1) - distance);
    if (eta > 4.0)
        eta = 4.0;
    else if (eta < -4.0)
        eta = -4.0;

    // 直線に対するロボットの向き
    double phai = -th;
    while (phai <= -M_PI || M_PI <= phai)
    {
        if (phai <= -M_PI)
            phai = phai + 2 * M_PI;
        else
            phai = phai - 2 * M_PI;
    }

    // 目標となるロボットの角速度と現在の角速度の差
    double w_diff = w0;

    // 角速度
    double w = w0 + (-k_eta * eta - k_phai * phai - k_w * w_diff) * 0.001;
    if (w > w_max)
        w = w_max;
    else if (w < -w_max)
        w = -w_max;

    std::cout << "eta: " << eta << "  phai; " << phai << "  w_diff:" << w_diff << std::endl;
    std::cout << "------------------------------" << std::endl;

    // 送信する値
    pub_msg.linear.x = max_v;
    pub_msg.linear.y = 0.0;
    pub_msg.linear.z = 0.0;
    pub_msg.angular.x = 0.0;
    pub_msg.angular.y = 0.0;
    pub_msg.angular.z = w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_node");

    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_callback);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    pub_msg.linear.x = 0.0;
    pub_msg.angular.z = 0.0;

    pos.pose.pose.position.x = 0.0;
    pos.pose.pose.position.y = 0.0;

    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_ang = 0.0;

    ros::Rate loop_rate(10);
    sleep(1);

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        wall2line();
        line_LC(line_x, line_y, line_theta);
        if (near_position(goal_x, goal_y))
        {
            pub_msg.linear.x = 0.0;
            pub_msg.angular.z = 0.0;
            count = 0;
            state = 0;
            std::cout << "finish!" << std::endl;
            return 0;
        }
        else if (count == 250)
        {
            pub_msg.linear.x = 0.0;
            pub_msg.angular.z = 0.0;
            count = 0;
            state = 0;
            std::cout << "20 seconds have passed. finish!" << std::endl;
            return 0;
        }
        if (state != 0)
            count++;
        pub.publish(pub_msg);
        loop_rate.sleep();
    }

    return 0;
}