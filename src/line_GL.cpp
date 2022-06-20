#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "math.h"

nav_msgs::Odometry pos;
geometry_msgs::Twist pub_msg;
double theta_rad = 0.0; // 追従直線の傾き
double double_x = 0.0;  // 追従直線が通る点Pのx
double double_y = 0.0;  // 追従直線が通る点Pのy
double max_v = 0.3;     // 速度
double goal_x = 0.0;    // ゴールのx
double goal_y = 0.0;    // ゴールのy

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos.header = msg->header;
    pos.child_frame_id = msg->child_frame_id;
    pos.pose = msg->pose;
    pos.twist = msg->twist;
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

// 直線追従
void line_GL(double x, double y, double th)
{
    // 制御のパラメータ(調整必須)
    const double k_eta = 400.0;
    const double k_phai = 300.0;
    const double k_w = 200.0;

    // 速度と角速度の最大値
    const double w_max = 0.2;

    tf::Quaternion quat(pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 現在のロボットの位置、姿勢
    double x0 = pos.pose.pose.position.x;
    double y0 = pos.pose.pose.position.y;
    double theta = yaw;

    // 現在の角速度
    double w0 = pos.twist.twist.angular.z;

    // ロボットと直線の距離
    double eta = 0;
    if (th == M_PI / 2.0)
        eta = -(x0 - x);
    else if (th == -M_PI / 2.0)
        eta = x0 - x;
    else if (abs(th) < M_PI / 2.0)
        eta = (-tan(th) * x0 + y0 - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    else
        eta = -(-tan(th) * x0 + y0 - y + x * tan(th)) / sqrt(tan(th) * tan(th) + 1);
    if (eta > 4.0)
        eta = 4.0;
    else if (eta < -4.0)
        eta = -4.0;

    // 直線に対するロボットの向き
    double phai = theta - th;
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
    std::cout << "(x,y,theta) = (" << x0 << "," << y0 << "," << theta << ")" << std::endl;
    std::cout << "------------------------------" << std::endl;

    // 送信する値
    pub_msg.linear.x = max_v;
    pub_msg.linear.y = 0.0;
    pub_msg.linear.z = 0.0;
    pub_msg.angular.x = 0.0;
    pub_msg.angular.y = 0.0;
    pub_msg.angular.z = w;
}

//　指定した座標付近で停止
int near_position(double goal_x, double goal_y)
{
    double difx = pos.pose.pose.position.x - goal_x;
    double dify = pos.pose.pose.position.y - goal_y;
    return (sqrt(difx * difx + dify * dify) < 0.2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    pnh.getParam("point_x_double", double_x);
    pnh.getParam("point_y_double", double_y);
    pnh.getParam("line_theta_radian", theta_rad);
    pnh.getParam("goal_x_double", goal_x);
    pnh.getParam("goal_y_double", goal_y);
    pnh.getParam("max_speed", max_v);

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

        line_GL(double_x, double_y, theta_rad);
        if (near_position(goal_x, goal_y))
        {
            pub_msg.linear.x = 0.0;
            pub_msg.angular.z = 0.0;
            count = 0;
            std::cout << "finish!" << std::endl;
            return 0;
        }
        else if (count == 200)
        {
            pub_msg.linear.x = 0.0;
            pub_msg.angular.z = 0.0;
            count = 0;
            std::cout << "20 seconds have passed. finish!" << std::endl;
            return 0;
        }
        count++;
        pub.publish(pub_msg);
        loop_rate.sleep();
    }

    return 0;
}