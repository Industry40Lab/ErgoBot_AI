#include "rclcpp/rclcpp.hpp"
#include "arm_controller/moverobotclass.hpp"

class PersonalNode : public rclcpp::Node{

    public:
        PersonalNode(rclcpp::NodeOptions& node_opt):Node("move_robot_node",node_opt){
            RCLCPP_INFO(this->get_logger(),"personal node has been initialized");
            

        }
};

int main(int argc, char **argv){

    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<PersonalNode> node = std::make_shared<PersonalNode>(node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();


    arm_controller::MoveRobotClass ur5e{"ur_manipulator", node};
    ur5e.getPlanningFrame();
    std::vector<double> jntnow{};
    jntnow = ur5e.getCurrentJointValues("Rad");
    ur5e.getPlanningGroupes();
    // std::vector<double> jntgoal{1.35418 + 0.1, -0.904172, 0.912273, -1.69176, 4.2133, 3.02263};
    // ur5e.goToJointGoal(jntgoal,"Rad");
    geometry_msgs::msg::PoseStamped current_pose = ur5e.getCurrentPose();
    // geometry_msgs::msg::Pose desiredpose = current_pose.pose;
    // std::cout<<"current Z value: "<<current_pose.pose.position.z<<std::endl;
    // desiredpose.position.z = current_pose.pose.position.z + 0.1;
    // std::cout<< "orientation "<< current_pose.pose.orientation.x<<""<<current_pose.pose.orientation.y<<""<<current_pose.pose.orientation.z<<current_pose.pose.orientation.w<<std::endl;
    // // desiredpose.orientation.w = 1.0;
    // // desiredpose.position.x =-0.0176;
    // // desiredpose.position.y = 0.05900;
    // // desiredpose.position.z = 0.21278+0.01;
    // // ur5e.move_group.setPlanningTime(5);
    // // ur5e.goToPoseGoal(desiredpose);
    // //jntnow = ur5e.getCurrentJointValues("Deg");
    // // std::vector<double> rpy = ur5e.move_group.getCurrentRPY();
    // // for (auto const  rr:rpy){
    // //         std::cout<<rr<<std::endl;
    // // }
    // std::cout<<"end effector link: "<< ur5e.move_group.getEndEffectorLink()<<std::endl;
    // std::cout<<"end effector: "<< ur5e.move_group.getEndEffector()<<std::endl;

    // std::cout<<"Pose reference frame is: "<<ur5e.move_group.getPoseReferenceFrame()<<std::endl;   
    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // waypoints.push_back(desiredpose);
    // moveit_msgs::msg::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // ur5e.move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    // // RCLCPP_INFO(node, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    // ur5e.move_group.execute(trajectory);
    // // rclcpp::spin(node);
    // // rclcpp::shutdown();
    return 0;
    
}