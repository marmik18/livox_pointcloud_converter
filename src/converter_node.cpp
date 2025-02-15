#include "livox_pointcloud_converter/converter.hpp"

class PointCloudConverter : public rclcpp::Node {
public:
    PointCloudConverter() : Node("livox_pointcloud_converter") {
        RCLCPP_INFO(this->get_logger(), "PointCloud Converter Node Started");

        // Subscribe to input topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/input_cloud", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudConverter::pointCloudCallback, this, std::placeholders::_1));

        // Publish converted cloud
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output_cloud", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        convertPointCloudCPU(msg, output_msg);
        publisher_->publish(*output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

// Convert function using OpenMP for speed
void convertPointCloudCPU(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg,
                          sensor_msgs::msg::PointCloud2::SharedPtr output_msg) {
    pcl::PointCloud<PointXYZRTLT> pcl_input;
    pcl::fromROSMsg(*input_msg, pcl_input);

    int num_points = pcl_input.size();
    pcl::PointCloud<pcl::PointXYZI> pcl_output;
    pcl_output.points.resize(num_points);

    // Parallelized processing usin OpenMP
    #pragma omp parallel for
    for (int i = 0; i < num_points; i++) {
        pcl_output.points[i].x = pcl_input.points[i].x;
        pcl_output.points[i].y = pcl_input.points[i].y;
        pcl_output.points[i].z = pcl_input.points[i].z;
        pcl_output.points[i].intensity = pcl_input.points[i].intensity;
    }

    pcl::toROSMsg(pcl_output, *output_msg);
    output_msg->header = input_msg->header;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudConverter>());
    rclcpp::shutdown();
    return 0;
}
