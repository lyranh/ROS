#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "camera_info_display_depth");
    ros::NodeHandle nh;

    // 创建ROSbag对象并打开文件
    rosbag::Bag bag;
    bag.open("/home/final/Desktop/catkin_ws/all.bag", rosbag::bagmode::Read);

    // 定义需要读取的话题
    std::vector<std::string> topics = {"/camera/depth/camera_info ", "/camera/depth/image_rect_raw"};

    // 创建ROSbag视图对象，并设置话题过滤器
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // 创建OpenCV窗口
    cv::namedWindow("Camera Image");

    // 遍历ROSbag中的消息
    for (const rosbag::MessageInstance& msg : view)
    {
        // 检查消息类型是否为CameraInfo
        if (msg.getDataType() == "sensor_msgs/CameraInfo")
        {
            sensor_msgs::CameraInfo::ConstPtr camera_info = msg.instantiate<sensor_msgs::CameraInfo>();

            // 在命令行中显示CameraInfo信息
            ROS_INFO_STREAM("CameraInfo: " << *camera_info);
        }
        // 检查消息类型是否为图像数据
        else if (msg.getDataType() == "sensor_msgs/Image")
        {
            sensor_msgs::Image::ConstPtr image = msg.instantiate<sensor_msgs::Image>();

            // 将图像数据转换为OpenCV格式
            cv_bridge::CvImagePtr cv_image;
            try
            {
                cv_image = cv_bridge::toCvCopy(image, image->encoding);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                continue;
            }

            // 显示图像
            cv::imshow("Camera Image", cv_image->image);

            // 等待用户按下ESC键退出
            if (cv::waitKey(1) == 27)
                break;
        }
    }

    // 关闭ROSbag文件
    bag.close();

    return 0;
}
