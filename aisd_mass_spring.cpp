#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class MassSpringSim : public rclcpp::Node
{
public:
    MassSpringSim() : Node("mass_spring_sim")
    {
        // --- 1. Initialize Parameters ---
        mass_ = 1.0;          // Mass (kg)
        k_ = 5.0;             // Spring stiffness (N/m)
        b_ = 0.2;             // Damping coefficient (N*s/m)
        dt_ = 0.02;           // Time step (50Hz)

        position_ = 2.0;      // Start with displacement
        velocity_ = 0.0;

        // --- 2. Publishers ---
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);

        position_pub_ = this->create_publisher<std_msgs::msg::Float32>("mass_position", 10);
        ke_pub_ = this->create_publisher<std_msgs::msg::Float32>("kinetic_energy", 10);
        pe_pub_ = this->create_publisher<std_msgs::msg::Float32>("potential_energy", 10);
        total_e_pub_ = this->create_publisher<std_msgs::msg::Float32>("total_energy", 10);

        // --- 3. Timer ---
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), 
            std::bind(&MassSpringSim::update_physics, this));

        RCLCPP_INFO(this->get_logger(), "Mass-Spring Simulation Started (Wall at -4.0).");
    }

private:
    void update_physics()
    {
        // --- PHYSICS ENGINE ---
        double f_spring = -k_ * position_;
        double f_damping = -b_ * velocity_;
        double f_net = f_spring + f_damping;

        double acceleration = f_net / mass_;
        velocity_ += acceleration * dt_;
        position_ += velocity_ * dt_;

        // --- ENERGY CALCULATION ---
        double ke = 0.5 * mass_ * std::pow(velocity_, 2);
        double pe = 0.5 * k_ * std::pow(position_, 2);
        double total_e = ke + pe;

        // --- PUBLISH DATA ---
        auto publish_float = [&](rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub, double val) {
            std_msgs::msg::Float32 msg;
            msg.data = val;
            pub->publish(msg);
        };

        publish_float(position_pub_, position_);
        publish_float(ke_pub_, ke);
        publish_float(pe_pub_, pe);
        publish_float(total_e_pub_, total_e);

        publish_markers();
    }

    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        auto current_time = this->get_clock()->now();

        // --- Marker 1: Fixed Point (Cube) ---
        // MOVED TO -4.0 TO PREVENT OVERLAP
        visualization_msgs::msg::Marker fixed_marker;
        fixed_marker.header.frame_id = "map";
        fixed_marker.header.stamp = current_time;
        fixed_marker.id = 0;
        fixed_marker.type = visualization_msgs::msg::Marker::CUBE;
        fixed_marker.action = visualization_msgs::msg::Marker::ADD;
        fixed_marker.pose.position.x = -4.0; 
        fixed_marker.pose.position.y = 0.0;
        fixed_marker.pose.position.z = 0.0;
        fixed_marker.scale.x = 0.2; fixed_marker.scale.y = 0.2; fixed_marker.scale.z = 0.2;
        fixed_marker.color.a = 1.0; fixed_marker.color.g = 1.0; // Green
        marker_array.markers.push_back(fixed_marker);

        // --- Marker 2: Mass (Sphere) ---
        visualization_msgs::msg::Marker mass_marker;
        mass_marker.header.frame_id = "map";
        mass_marker.header.stamp = current_time;
        mass_marker.id = 1;
        mass_marker.type = visualization_msgs::msg::Marker::SPHERE;
        mass_marker.action = visualization_msgs::msg::Marker::ADD;
        mass_marker.pose.position.x = position_; // Moves based on physics
        mass_marker.pose.position.y = 0.0;
        mass_marker.pose.position.z = 0.0;
        mass_marker.scale.x = 0.3; mass_marker.scale.y = 0.3; mass_marker.scale.z = 0.3;
        mass_marker.color.a = 1.0; mass_marker.color.r = 1.0; // Red
        marker_array.markers.push_back(mass_marker);

        // --- Marker 3: Spring (Line Strip) ---
        visualization_msgs::msg::Marker spring_marker;
        spring_marker.header.frame_id = "map";
        spring_marker.header.stamp = current_time;
        spring_marker.id = 2;
        spring_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        spring_marker.action = visualization_msgs::msg::Marker::ADD;
        spring_marker.scale.x = 0.05; 
        spring_marker.color.a = 1.0; spring_marker.color.b = 1.0; // Blue
        
        // Connects Wall (-4.0) to Mass (position_)
        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = -4.0; 
        p_start.y = 0.0; p_start.z = 0.0;
        p_end.x = position_; 
        p_end.y = 0.0; p_end.z = 0.0;
        
        spring_marker.points.push_back(p_start);
        spring_marker.points.push_back(p_end);
        marker_array.markers.push_back(spring_marker);

        marker_pub_->publish(marker_array);
    }

    double mass_, k_, b_, dt_;
    double position_, velocity_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ke_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pe_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr total_e_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MassSpringSim>());
    rclcpp::shutdown();
    return 0;
}
