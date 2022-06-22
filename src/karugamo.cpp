#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "math.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"

int flag = 0;

nav_msgs::Odometry pos;
geometry_msgs::Twist pub_msg;
sensor_msgs::LaserScan scan;
geometry_msgs::PoseStamped goal_msg; // 追従目標点に関するmsg
double scan_coord[726][2];           // scanデータの座標
int cluster[726];                    // クラスタ情報
double goal_point[2] = {100, 100};   // 追従目標のクラスタ中心
double pre_goal_cluster[726][2];     // 追従目標の計算に用いている点群
int num_pre_goal = 0;                // 追従目標の計算に用いている点の数

// urgのコールバック関数
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

// オドメトリのコールバック関数
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

// クラスタ作成
void make_cluster()
{
    scan2coord();
    for (int i = 0; i < 726; i++)
        cluster[i] = 0; // クラスタの初期化
    int c_num = 0;
    int data_count = 0;
    for (int i = 0; i < 726; i++)
    {
        if (cluster[i] == 0 && scan.ranges[i] != 0)
        {
            c_num++;
            data_count++;
            cluster[i] = c_num;
            int prev = i;
            for (int j = i + 1; j < 726; j++)
            {
                if (scan.ranges[j] != 0 && cluster[j] == 0)
                {
                    if ((pow(scan_coord[prev][0] - scan_coord[j][0], 2) + pow(scan_coord[prev][1] - scan_coord[j][1], 2)) < 0.005625)
                    {
                        cluster[j] = c_num;
                        prev = j;
                        data_count++;
                    }
                }
            }
        }
        if (data_count == 726)
            break;
    }
}

// クラスタの数
int count_cluster()
{
    int count_c = 0;
    for (int i = 0; i < 726; i++)
    {
        if (cluster[i] != 0)
            count_c = cluster[i];
    }
    return count_c;
}

// 追従対象の初期化(flag==0で使用)
void init_goal_cluster()
{
    int cluster_num = count_cluster(); // クラスターの総数
    bool use_cluster[cluster_num];
    num_pre_goal = 0;
    goal_point[0] = 0;
    goal_point[1] = 0;

    // ロボットの正面0.6m, 左右0.2m以内に1点でも属するクラスタを不採用とする
    // それ以外のクラスタを採用する
    for (int i = 0; i < cluster_num; i++)
    {
        use_cluster[i] = true;
        for (int j = 0; j < 726; j++)
        {
            if (cluster[j] - 1 == i)
            {
                if (scan_coord[j][0] > 1.5 || scan_coord[j][0] < 0 || abs(scan_coord[j][1]) > 0.25)
                {
                    use_cluster[cluster[j] - 1] = false;
                    break;
                }
            }
        }
    }

    // 採用したクラスタから追従先を登録
    for (int j = 0; j < 726; j++)
    {
        pre_goal_cluster[j][0] = 0;
        pre_goal_cluster[j][1] = 0;
        if (use_cluster[cluster[j] - 1])
        {
            flag = 1;
            pre_goal_cluster[num_pre_goal][0] = scan_coord[j][0];
            pre_goal_cluster[num_pre_goal][1] = scan_coord[j][1];

            goal_point[0] += scan_coord[j][0];
            goal_point[1] += scan_coord[j][1];
            num_pre_goal++;
        }
    }

    if (flag != 0)
    {
        goal_point[0] /= num_pre_goal;
        goal_point[1] /= num_pre_goal;
    }
    else
    {
        flag = 0;
        goal_point[0] = 0;
        goal_point[1] = 0;
    }
    std::cout << "親を再登録" << std::endl;
    std::cout << "init : (" << goal_point[0] << ", " << goal_point[1] << ")" << std::endl;
}

// 追従するクラスタの登録(flag==1で使用)
void resister_cluster()
{
    int cluster_num = count_cluster(); // クラスターの総数
    int num_goal = 0;                  // 追従目標点の計算に用いたスキャンの数
    bool use_cluster[cluster_num];     //各クラスタを追従目標点の計算に用いるか
    flag = 0;

    // 前回の追従目標に用いた点群から0.3m以上離れている点が1点でも属するクラスタを不採用とする
    // それ以外のクラスタを採用する
    for (int i = 0; i < cluster_num; i++)
    {
        use_cluster[i] = true;
    }
    for (int i = 0; i < 726; i++)
    {
        for (int j = 0; j < num_pre_goal; j++)
        {
            if (pow(scan_coord[i][0] - pre_goal_cluster[j][0], 2) + pow(scan_coord[i][1] - pre_goal_cluster[j][1], 2) > 0.16)
            {
                use_cluster[cluster[i] - 1] = false;
                break;
            }
        }
    }

    num_pre_goal = 0;

    goal_point[0] = 0;
    goal_point[1] = 0;
    // 採用したクラスタから追従先を登録
    for (int j = 0; j < 726; j++)
    {
        pre_goal_cluster[j][0] = 0;
        pre_goal_cluster[j][1] = 0;
        if (use_cluster[cluster[j] - 1])
        {
            flag = 1;
            pre_goal_cluster[num_pre_goal][0] = scan_coord[j][0];
            pre_goal_cluster[num_pre_goal][1] = scan_coord[j][1];

            goal_point[0] += scan_coord[j][0];
            goal_point[1] += scan_coord[j][1];
            num_pre_goal++;
        }
    }

    if (flag != 0)
    {
        goal_point[0] /= num_pre_goal;
        goal_point[1] /= num_pre_goal;
    }

    // std::cout << "register : (" << goal_point[0] << ", " << goal_point[1] << ")" << std::endl;
}

// 追従目標点の可視化(ロボット座標系での可視化)
// PoseStampedのPublisher必須
void show_point_LC(double x, double y)
{
    // 目標点の方向
    double phai = atan2(y, x);

    // ロボットの目標点のパブリッシュ
    goal_msg.header.frame_id = "laser";
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.position.z = 0;
}

// 追従目標点へ向かうための速度指令(ロボット座標系で指令)
void go_point_LC(double x, double y)
{
    // パラメータ
    double k_v = 3.0;
    double k_w = 0.8;
    //追従対象との距離
    double distance = 0.5;

    // 速度と角速度の最大値
    const double v_max = 0.8;
    const double w_max = 1.2;

    tf::Quaternion quat(pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // ロボットと点との距離
    double eta = sqrt(pow(x, 2) + pow(y, 2));
    double phai = atan2(y, x);

    double v = k_v * (eta - distance);
    double w = k_w * phai;

    if (v > v_max)
        v = v_max;
    else if (v < -v_max)
        v = -v_max;
    if (abs(v) < 0.1)
        v = 0;

    if (w > w_max)
        w = w_max;
    else if (w < -w_max)
        w = -w_max;
    if (abs(w) < 0.1)
        w = 0;

    // 送信する値
    pub_msg.linear.x = v;
    pub_msg.linear.y = 0.0;
    pub_msg.linear.z = 0.0;
    pub_msg.angular.x = 0.0;
    pub_msg.angular.y = 0.0;
    pub_msg.angular.z = w;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "karugamo_node");

    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_callback);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_point", 1000);

    pub_msg.linear.x = 0.0;
    pub_msg.angular.z = 0.0;

    ros::Rate loop_rate(10);
    sleep(1);

    int pre_cluster_num = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        make_cluster();
        int cluster_num = count_cluster();
        // if (cluster_num != pre_cluster_num)
        //     std::cout << "cluster num = " << cluster_num << std::endl;

        // クラスタをrvizに描画擦るための記述
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.resize(cluster_num);
        for (int i = 0; i < cluster_num; i++)
        {
            marker_array.markers[i].header.frame_id = "laser";
            marker_array.markers[i].header.stamp = ros::Time::now();
            marker_array.markers[i].ns = "cluster";
            marker_array.markers[i].id = i;
            marker_array.markers[i].lifetime = ros::Duration();

            marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
            marker_array.markers[i].action = visualization_msgs::Marker::ADD;

            marker_array.markers[i].scale.x = 0.2;
            marker_array.markers[i].scale.y = 0.2;
            marker_array.markers[i].scale.z = 0.4;

            double cluster_coord[2] = {0.0, 0.0};
            int data_count = 0;
            for (int j = 0; j < 726; j++)
            {
                if (cluster[j] == i + 1)
                {
                    cluster_coord[0] += scan_coord[j][0];
                    cluster_coord[1] += scan_coord[j][1];
                    data_count++;
                }
            }
            cluster_coord[0] = cluster_coord[0] / static_cast<double>(data_count);
            cluster_coord[1] = cluster_coord[1] / static_cast<double>(data_count);

            marker_array.markers[i].pose.position.x = cluster_coord[0];
            marker_array.markers[i].pose.position.y = cluster_coord[1];
            marker_array.markers[i].pose.position.z = 0.0;

            marker_array.markers[i].pose.orientation.x = 0.0;
            marker_array.markers[i].pose.orientation.y = 0.0;
            marker_array.markers[i].pose.orientation.z = 0.0;
            marker_array.markers[i].pose.orientation.w = 1.0;

            marker_array.markers[i].color.r = 0.0f;
            marker_array.markers[i].color.g = 1.0f;
            marker_array.markers[i].color.b = 0.0f;
            marker_array.markers[i].color.a = 1.0f;
        }
        marker_pub.publish(marker_array);

        if (flag == 0)
        {
            init_goal_cluster();
            show_point_LC(goal_point[0], goal_point[1]);
        }
        else
        {
            resister_cluster();
            show_point_LC(goal_point[0], goal_point[1]);
            go_point_LC(goal_point[0], goal_point[1]);
        }
        if (flag == 0)
        {
            pub_msg.linear.x = 0.0;
            pub_msg.linear.y = 0.0;
            pub_msg.linear.z = 0.0;
            pub_msg.angular.x = 0.0;
            pub_msg.angular.y = 0.0;
            pub_msg.angular.z = 0.0;
        }
        pub.publish(pub_msg);
        goal_pub.publish(goal_msg);

        loop_rate.sleep();
        pre_cluster_num = cluster_num;
    }
    return 0;
}