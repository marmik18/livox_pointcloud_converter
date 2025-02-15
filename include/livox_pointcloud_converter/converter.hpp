#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <omp.h>

struct PointXYZRTLT {
    float x, y, z;
    float intensity;    // Intensity (R)
    uint8_t tag;        // Tag (T)
    uint8_t line;       // Line (L)
    double timestamp;   // Timestamp (T)
};

// Register the struct as a PCL point type
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRTLT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity) // Intensity is renamed
                                  (uint8_t, tag, tag)
                                  (uint8_t, line, line)
                                  (double, timestamp, timestamp)
)

void convertPointCloudCPU(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg,
                          sensor_msgs::msg::PointCloud2::SharedPtr output_msg);