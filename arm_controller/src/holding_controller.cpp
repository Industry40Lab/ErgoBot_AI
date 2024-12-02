#include <cstdio>   
#include "rclcpp/rclcpp.hpp"
#include "arm_controller/moverobotclass.hpp"
#include "robot_interface/srv/gripper_service.hpp"
#include <memory>
#include <chrono>
#include <thread>
#include "std_msgs/msg/string.hpp"
#include <string>
#include <vector>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "geometry_msgs/msg/pose.hpp"

class HoldingController : public rclcpp::Node{

    public:
        HoldingController(rclcpp::NodeOptions& node_opt):Node("holding_controller",node_opt){
            RCLCPP_INFO(this->get_logger(),"personal node has been initialized");
        }
};


class NormalNode : public rclcpp::Node{

    public:
        NormalNode(std::shared_ptr<arm_controller::MoveRobotClass>  robot):Node("subscriber_node"), robot{robot} {
            RCLCPP_INFO(this->get_logger(),"Normal node has been initialized");
            subscription_ = this->create_subscription<std_msgs::msg::String> (
            "/classified_order", 1,std::bind(&NormalNode::user_command_analyzis,this,std::placeholders::_1));  
            client_gripper = this->create_client<robot_interface::srv::GripperService>("/gripper_service");  
            request_gripper = std::make_shared<robot_interface::srv::GripperService::Request>();
        }

    void send_grip_request(int position, int speed,int force){
    
        request_gripper->position = position;
        request_gripper->speed    = speed;
        request_gripper->force    = force;
        client_gripper->async_send_request(request_gripper,std::bind(&NormalNode::callback_grip_request,this,std::placeholders::_1));
        
    }

    
    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::shared_ptr<arm_controller::MoveRobotClass> robot;

    std::shared_ptr<rclcpp::Client<robot_interface::srv::GripperService>> client_gripper;
    std::shared_ptr<robot_interface::srv::GripperService::Request> request_gripper;


    void user_command_analyzis(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string input = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", input.c_str());
        // Split the input string by comma
        std::string part1, part2;
        std::size_t pos = input.find(',');

        if (pos != std::string::npos) {
            part1 = input.substr(0, pos);
            part2 = input.substr(pos + 1);

            RCLCPP_INFO(this->get_logger(), "Part 1: '%s'", part1.c_str());
            RCLCPP_INFO(this->get_logger(), "Part 2: '%s'", part2.c_str());

            float x,y,z;
            x = 0;
            y = 0;
            z = 0;

            if (part1 == "new_pose"){
                // std::stringstream values(part2);
                // std::vector<std::string> segmented_numbers;
                // boost::split(segmented_numbers, part2, boost::is_any_of("_"));
                // while(std::getline(values, segmented_numbers, '_'))
                // {
                //     seglist.push_back(segmented_numbers);
                // }

                if (part2 == "up")
                    z = 0.1;

                if (part2 == "down")
                    z = -0.1;

                if (part2 == "forward")
                    x = 0.1;

                if (part2 == "backward")
                    x = -0.1;

                if (part2 == "right")
                    y = 0.1;

                if (part2 == "left")
                    y = -0.1;


                geometry_msgs::msg::Pose update_pose;


                update_pose.position.x = x;
                update_pose.position.y = y;
                update_pose.position.z = z;

                move_robot_update(update_pose);                
            }

            if (part1 == "gripper"){
                if (part2 == "open")
                    open_gripper();            
            }




        } else {
            RCLCPP_WARN(this->get_logger(), "No comma found in the input string.");
        }
    }


    void move_robot_update(geometry_msgs::msg::Pose msg){
        

        geometry_msgs::msg::PoseStamped current_pose = robot->getCurrentPose();
        geometry_msgs::msg::Pose desiredpose;

        std::cout << "Previous Position x: " << current_pose.pose.position.x << std::endl;
        std::cout << "Previous Position y: " << current_pose.pose.position.y << std::endl;
        std::cout << "Previous Position z: " << current_pose.pose.position.z << std::endl;



        desiredpose.orientation = current_pose.pose.orientation;
        desiredpose.position.x = current_pose.pose.position.x + msg.position.x;
        desiredpose.position.y = current_pose.pose.position.y + msg.position.y;
        desiredpose.position.z = current_pose.pose.position.z + msg.position.z;

        std::cout << "Position x: " << msg.position.x << std::endl;
        std::cout << "Position y: " << msg.position.y << std::endl;
        std::cout << "Position z: " << msg.position.z << std::endl;

        std::cout << "New Position x: " << desiredpose.position.x << std::endl;
        std::cout << "New Position y: " << desiredpose.position.y << std::endl;
        std::cout << "New Position z: " << desiredpose.position.z << std::endl;

        

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(desiredpose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        robot->move_group.setPlanningTime(5);

        double jump_threshold = 5.0;
        double eef_step = 0.01;

        robot->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        robot->move_group.execute(trajectory);

        RCLCPP_INFO(this->get_logger(), "execution check1:");
        std::this_thread::sleep_for(std::chrono::seconds(1));

    }

    
    void callback_grip_request(std::shared_future<std::shared_ptr<robot_interface::srv::GripperService::Response>> grip_response){
        auto response = grip_response.get();
        RCLCPP_INFO(this->get_logger(), "Received response from gripper: = %s", response->response.c_str());

    }

    void open_gripper(){
        this->send_grip_request(20,50,5); // open the gripper
    }
};


int main(int argc, char **argv){

    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<HoldingController> node = std::make_shared<HoldingController>(node_options);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    std::shared_ptr<arm_controller::MoveRobotClass> ur5e = std::make_shared<arm_controller::MoveRobotClass>("ur_manipulator", node);

    std::shared_ptr<NormalNode> controller_node = std::make_shared<NormalNode>(ur5e);



    std::cout<<"Main thread is blocked "<<std::endl;
    rclcpp::executors::SingleThreadedExecutor controller_executor;
    controller_executor.add_node(controller_node);
    controller_executor.spin();

    return 0;
    
}