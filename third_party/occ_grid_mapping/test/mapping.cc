// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <occ_grid_mapping/grid_map.h>
#include <occ_grid_mapping/grid_mapper.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

/* Global */
GridMap* g_map;
GridMapper* g_gmapper;
tf::StampedTransform g_transform;

ros::Subscriber g_odom_suber;
ros::Publisher g_map_puber;
Pose2d g_robot_pose;
Pose2d g_lidar_pose;

void odometryCallback ( const nav_msgs::OdometryConstPtr& odom );
void laserCallback ( const sensor_msgs::LaserScanConstPtr& scan );
void mapCallback ( const nav_msgs::OccupancyGridConstPtr& map );

int main ( int argc, char **argv )
{
    /***** 初始化ROS *****/
    ros::init ( argc, argv, "GridMapping" );
    ros::NodeHandle nh;

    /***** 加载参数 *****/
    int map_sizex, map_sizey, map_initx, map_inity;
    double map_cell_size;
    /* TODO 错误处理 */
    nh.getParam ( "/mapping/map/sizex", map_sizex );
    nh.getParam ( "/mapping/map/sizey", map_sizey );
    nh.getParam ( "/mapping/map/initx", map_initx );
    nh.getParam ( "/mapping/map/inity", map_inity );
    nh.getParam ( "/mapping/map/cell_size", map_cell_size );

    Pose2d T_r_l;
    double x, y, theta;
    double P_occ, P_free, P_prior;
    /* TODO 错误处理 */
    nh.getParam ( "/mapping/robot_laser/x", x );
    nh.getParam ( "/mapping/robot_laser/y", y );
    nh.getParam ( "/mapping/robot_laser/theta", theta );
    T_r_l = Pose2d ( x, y, theta );

    nh.getParam ( "/mapping/sensor_model/P_occ", P_occ );
    nh.getParam ( "/mapping/sensor_model/P_free", P_free );
    nh.getParam ( "/mapping/sensor_model/P_prior", P_prior );
    
    /* 地图保存地址 */
    std::string map_image_save_dir, map_config_save_dir;
    nh.getParam ( "/mapping/map_image_save_dir", map_image_save_dir );
    nh.getParam ( "/mapping/map_config_save_dir", map_config_save_dir );

    // Map subscriber
    ros::Subscriber map_sub = nh.subscribe ( "/map", 1, mapCallback );
    /***** 初始化地图和构图器 *****/
    // g_map = new GridMap ( map_sizex, map_sizey,  map_initx, map_inity, map_cell_size );
    while (g_map == NULL)
    {
        // wait for map
        ros::spinOnce();
    }

    g_gmapper = new GridMapper ( g_map, T_r_l, P_occ, P_free, P_prior );

    /***** 初始Topic *****/
    g_odom_suber = nh.subscribe ( "/gazebo/ground_truth/state", 1, odometryCallback );
    ros::Subscriber laser_suber = nh.subscribe ( "/front/scan", 1, laserCallback );
    g_map_puber = nh.advertise<nav_msgs::OccupancyGrid> ( "map1", 1 );
    

    
    ros::spin();

    /* TODO 保存地图 */
    g_map->saveMap(map_image_save_dir, map_config_save_dir);
    
    std::cout << "\nMap saved\n";
}

void odometryCallback ( const nav_msgs::OdometryConstPtr& odom )
{
    /* 获取机器人姿态 */
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    double theta = tf::getYaw ( odom->pose.pose.orientation );
    g_robot_pose = Pose2d ( x, y, theta );
}

void laserCallback ( const sensor_msgs::LaserScanConstPtr& scan )
{
    // get lidar pose
    try
    {
        tf::TransformListener listener;
        listener.waitForTransform ( "map", "tim551", ros::Time ( 0 ), ros::Duration ( 3.0 ) );
        listener.lookupTransform ( "map", "tim551", ros::Time ( 0 ), g_transform );
    }
    catch ( tf::TransformException& ex )
    {
        ROS_ERROR ( "%s", ex.what() );
        return;
    }
    double x = g_transform.getOrigin().x();
    double y = g_transform.getOrigin().y();
    double theta = tf::getYaw ( g_transform.getRotation() );
    g_lidar_pose = Pose2d ( x, y, theta );


    /* 更新地图 */
    g_gmapper->updateMap ( scan, g_robot_pose );
    
    /* 用opencv图像显示地图 */
    cv::Mat map = g_map->toCvMat();
    cv::imshow ( "map", map );
    cv::waitKey ( 1 );
    
    /* 发布地图 */
    nav_msgs::OccupancyGrid occ_map;
    g_map->toRosOccGridMap ( "map", occ_map );
    g_map_puber.publish ( occ_map );
}

void mapCallback ( const nav_msgs::OccupancyGridConstPtr& map )
{
    /* 将全局地图信息放入g_map以初始化 */
    g_map = new GridMap ( *map );
}