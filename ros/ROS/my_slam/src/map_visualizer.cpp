#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

ros::Subscriber map_sub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  // 将地图数据转换为OpenCV图像格式
  cv::Mat map_image(map->info.height, map->info.width, CV_8UC1);
  for (int y = 0; y < map->info.height; ++y)
  {
    for (int x = 0; x < map->info.width; ++x)
    {
      int index = y * map->info.width + x;
      if (map->data[index] > 50)
      {
        map_image.at<uchar>(y, x) = 0; // 占据的像素为黑色
      }
      else if (map->data[index] >= 0 && map->data[index] <= 50)
      {
        map_image.at<uchar>(y, x) = 127; // 未知的像素为灰色
      }
      else
      {
        map_image.at<uchar>(y, x) = 255; // 自由的像素为白色
      }
    }
  }

  // 显示地图图像
  cv::Mat map_display;
  cv::cvtColor(map_image, map_display, CV_GRAY2BGR);
  cv::namedWindow("Map", cv::WINDOW_NORMAL);
  cv::imshow("Map", map_display);

  // 将窗口大小调整为800x600像素
  cv::resizeWindow("Map", 800, 600);

  cv::waitKey(1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_visualizer");
  ros::NodeHandle nh;

  map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);

  ros::spin();

  return 0;
}
