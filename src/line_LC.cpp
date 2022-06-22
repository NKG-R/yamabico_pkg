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
double scan_coord[726][2]; // scanデータの座標
double line_x = 0.0;       // 追従直線が通る点のx
double line_y = 0.0;       // 追従直線が通る点のy
double line_x2 = 0.0;
double line_y2 = 0.0;
double line_theta = 0.0; // 追従直線の角度
double max_v = 0.3;      // 速度
double k_eta = 400.0;
double k_phai = 300.0;
double k_w = 200.0;
double wall_distance = 0.8;

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
        double th = i * 255.0 / 726.0;
        th = th - 135.0; // 実機
        // th = th - 120.0; // シミュレータ
        double theta = th * M_PI / 180.0;
        if (!(scan.ranges[i] <= 5.0 && scan.ranges[i] >= 0.05))
            scan.ranges[i] = 100.0;
        scan_coord[i][0] = scan.ranges[i] * cos(theta);
        scan_coord[i][1] = scan.ranges[i] * sin(theta);
        i++;
    }
}

// 壁の直線を計算
void wall2line()
{
    scan2coord();
    // 直線が通る点を保存
    for (int i = -1; i < 2; i++)
    {
        line_x += scan_coord[640 + i][0];
        line_y += scan_coord[640 + i][1];
        line_x2 += scan_coord[610 + i][0];
        line_y2 += scan_coord[610 + i][1];
    }
    line_x /= 3.0;
    line_y /= 3.0;
    line_x2 /= 3.0;
    line_y2 /= 3.0;
    // 直線の傾き(角度)を保存
    if ((line_x2 - line_x) != 0)
        line_theta = atan((line_y2 - line_y) / (line_x2 - line_x));
    else
        line_theta = M_PI / 2.0;
}

// 直線追従
void line_LC(double x, double y, double th)
{
    // 制御のパラメータ(調整必須)

    // 角速度の最大値
    const double w_max = 0.5;

    // 現在の角速度
    double w0 = pos.twist.twist.angular.z;

    // ロボットと直線の距離
    double eta = 0;
    if (line_x == line_x2)
        eta = -line_x;
    else
        eta = -abs((line_y2 - line_y) / (line_x2 - line_x) * (-line_x) + line_y) / sqrt(pow(line_y2 - line_y, 2) / pow(line_x2 - line_x, 2) + 1);
    if (eta > 4.0)
        eta = 4.0;
    else if (eta < -4.0)
        eta = -4.0;

    if (eta > 0)
        eta -= wall_distance;
    else
        eta += wall_distance;

    // 直線に対するロボットの向き
    double phai = -th;

    // 角速度
    double w = w0 + (-k_eta * eta - k_phai * phai - k_w * w0) * 0.001;
    if (w > w_max)
        w = w_max;
    else if (w < -w_max)
        w = -w_max;

    std::cout << "(x1,y1) " << line_x << ", " << line_y << std::endl;
    std::cout << "eta: " << eta << "  phai; " << phai << "  w0:" << w0 << std::endl;
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
    ros::NodeHandle pnh("~");
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_callback);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    pnh.getParam("max_speed", max_v);
    pnh.getParam("gain_eta", k_eta);
    pnh.getParam("gain_phi", k_phai);
    pnh.getParam("gain_w", k_w);
    pnh.getParam("wall_d", wall_distance);

    pub_msg.linear.x = 0.0;
    pub_msg.angular.z = 0.0;

    ros::Rate loop_rate(10);
    sleep(1);

    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        wall2line();
        line_LC(line_x, line_y, line_theta);
        if (count == 300)
        {
            pub_msg.linear.x = 0.0;
            pub_msg.angular.z = 0.0;
            count = 0;
            std::cout << "30 seconds have passed. finish!" << std::endl;
            return 0;
        }
        count++;
        pub.publish(pub_msg);
        loop_rate.sleep();
    }

    return 0;
}