#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "aisd_mass_spring/srv/set_system_values.hpp"

using namespace std::chrono_literals;

class MassSpringSim : public rclcpp::Node
{
public:
    MassSpringSim() : Node("aisd_mass_spring")
    {
        // --- 1. Physics Parameters ---
        mass_ = 8.0;
        k_ = 5.0;
        b_ = 0.2;
        dt_ = 0.02;

        position_ = 0.0; // Start at equilibrium
        velocity_ = 0.0; // Start at rest

        // --- 2. Publishers ---
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
        
        // Energy Publishers
        ke_pub_ = this->create_publisher<std_msgs::msg::Float32>("kinetic_energy", 10);
        pe_pub_ = this->create_publisher<std_msgs::msg::Float32>("potential_energy", 10);
        total_e_pub_ = this->create_publisher<std_msgs::msg::Float32>("total_energy", 10);

        // --- 3. Force Service ---
        force_service_ = this->create_service<std_srvs::srv::Trigger>(
            "apply_force",
            std::bind(&MassSpringSim::apply_force_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // --- 4. Parameter Service (FIXED) ---
        // Fixed: Changed MassSpringNode to MassSpringSim
        param_service_ = this->create_service<aisd_mass_spring::srv::SetSystemValues>(
            "/set_system_params",
            std::bind(&MassSpringSim::update_params_callback, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // --- 5. Timer ---
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), 
            std::bind(&MassSpringSim::update_physics, this));

        RCLCPP_INFO(this->get_logger(), "Simulation Ready. Call /apply_force to kick the mass.");
        RCLCPP_INFO(this->get_logger(), "Service /set_system_params is ready.");
    }

private:
    // --- Member Variables ---
    double mass_, k_, b_, dt_; // NOTE: Variables are named k_ and b_
    double position_, velocity_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ke_pub_, pe_pub_, total_e_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr force_service_;
    rclcpp::Service<aisd_mass_spring::srv::SetSystemValues>::SharedPtr param_service_;

    // --- Callbacks ---

    // Fixed: Using correct variable names k_ and b_
    void update_params_callback(
        const std::shared_ptr<aisd_mass_spring::srv::SetSystemValues::Request> request,
        std::shared_ptr<aisd_mass_spring::srv::SetSystemValues::Response> response)
    {
        // Update physics variables
        k_ = request->k;
        b_ = request->b;

        // Log it
        RCLCPP_INFO(this->get_logger(), "Params Updated -> k: %.2f, b: %.2f", k_, b_);

        // Reply
        response->success = true;
        response->message = "Physics parameters updated.";
    }
        
    void apply_force_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused parameter
        
        // Apply a sudden "kick" (Impulse)
        velocity_ += 3.0; 

        response->success = true;
        response->message = "Perturbation applied! Velocity increased by 3.0 m/s";
        
        RCLCPP_INFO(this->get_logger(), "Service called: Applying perturbation force.");
    }

    void update_physics()
    {
        // Physics Engine
        double f_spring = -k_ * position_;
        double f_damping = -b_ * velocity_;
        double f_net = f_spring + f_damping;

        double acceleration = f_net / mass_;
        velocity_ += acceleration * dt_;
        position_ += velocity_ * dt_;

        // Energy Calculation
        double ke = 0.5 * mass_ * std::pow(velocity_, 2);
        double pe = 0.5 * k_ * std::pow(position_, 2);
        double total_e = ke + pe;

        // Publish Data
        auto publish_float = [&](auto pub, double val) {
            std_msgs::msg::Float32 msg; msg.data = val; pub->publish(msg);
        };
        publish_float(ke_pub_, ke);
        publish_float(pe_pub_, pe);
        publish_float(total_e_pub_, total_e);

        publish_markers();
    }

    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        auto time = this->get_clock()->now();

        // Marker 1: Wall (Fixed at -4.0)
        visualization_msgs::msg::Marker wall;
        wall.header.frame_id = "map"; wall.header.stamp = time; wall.id = 0;
        wall.type = visualization_msgs::msg::Marker::CUBE; wall.action = 0;
        wall.pose.position.x = -4.0; wall.scale.x = 0.2; wall.scale.y = 2.0; wall.scale.z = 2.0;
        wall.color.a = 1.0; wall.color.g = 1.0;
        marker_array.markers.push_back(wall);

        // Marker 2: Mass
        visualization_msgs::msg::Marker mass;
        mass.header.frame_id = "map"; mass.header.stamp = time; mass.id = 1;
        mass.type = visualization_msgs::msg::Marker::SPHERE; mass.action = 0;
        mass.pose.position.x = position_; 
        mass.scale.x = 0.5; mass.scale.y = 0.5; mass.scale.z = 0.5;
        mass.color.a = 1.0; mass.color.r = 1.0;
        marker_array.markers.push_back(mass);

        // Marker 3: Spring
        visualization_msgs::msg::Marker spring;
        spring.header.frame_id = "map"; spring.header.stamp = time; spring.id = 2;
        spring.type = visualization_msgs::msg::Marker::LINE_STRIP; spring.action = 0;
        spring.scale.x = 0.05; spring.color.a = 1.0; spring.color.b = 1.0;
        geometry_msgs::msg::Point p1, p2;
        p1.x = -4.0; p2.x = position_;
        spring.points.push_back(p1); spring.points.push_back(p2);
        marker_array.markers.push_back(spring);

        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MassSpringSim>());
    rclcpp::shutdown();
    return 0;
}
