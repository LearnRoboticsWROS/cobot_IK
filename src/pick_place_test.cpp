#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// tau = 1 rotation in radiants
const double tau = 2 * M_PI;
#include <std_msgs/Float64.h>

void close_gripper(moveit::planning_interface::MoveGroupInterface& move_gripper)
{
    move_gripper.setJointValueTarget("gripper_right_joint", 0.055);
    move_gripper.move();
}

void open_gripper(moveit::planning_interface::MoveGroupInterface& move_gripper)
{
    move_gripper.setJointValueTarget("gripper_right_joint", 0.0);
    move_gripper.move();
}

void pre_pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    geometry_msgs::Pose pick_position;
    
    tf2::Quaternion orientation;
    orientation.setRPY(tau/4, - tau/4, 0);
    pick_position.orientation = tf2::toMsg(orientation);
    pick_position.position.x = 1;
    pick_position.position.y = 0.15;
    pick_position.position.z = 0.5;
    move_group.setPoseTarget(pick_position);

    move_group.move();

}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    geometry_msgs::Pose pick_position;
    
    tf2::Quaternion orientation;
    orientation.setRPY(tau/4, - tau/4, 0);
    pick_position.orientation = tf2::toMsg(orientation);
    pick_position.position.x = 1;
    pick_position.position.y = 0.05;
    pick_position.position.z = 0.5;
    move_group.setPoseTarget(pick_position);

    move_group.move();

}

void place(moveit::planning_interface::MoveGroupInterface& move_group_place)
{
    geometry_msgs::Pose place_position;
    tf2::Quaternion orientation;
    orientation.setRPY(0, tau/4, 0);
    place_position.orientation = tf2::toMsg(orientation);
    place_position.position.x = 0;
    place_position.position.y = 1;
    place_position.position.z = 0.7;
    move_group_place.setPoseTarget(place_position);

    move_group_place.move();
}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    // Define primitive dimension, position of the table 1
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;
    // pose of table 1
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 1;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 1 to the scene
    collision_objects[0].operation = collision_objects[0].ADD;


    // Add the second table
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";

    // Define primitive dimension, position of the table 2
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;
    // pose of table 2
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 1;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the scene
    collision_objects[1].operation = collision_objects[1].ADD;

    // // Add the object to be picked
    // collision_objects[2].id = "object";
    // collision_objects[2].header.frame_id = "base_link";

    // // Define primitive dimension, position of the object
    // collision_objects[2].primitives.resize(1);
    // collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    // collision_objects[2].primitives[0].dimensions.resize(3);
    // collision_objects[2].primitives[0].dimensions[0] = 0.02;
    // collision_objects[2].primitives[0].dimensions[1] = 0.02;
    // collision_objects[2].primitives[0].dimensions[2] = 0.2;
    // // pose of object
    // collision_objects[2].primitive_poses.resize(1);
    // collision_objects[2].primitive_poses[0].position.x = 1;
    // collision_objects[2].primitive_poses[0].position.y = 0;
    // collision_objects[2].primitive_poses[0].position.z = 0.5;
    // collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // // Add tabe 2 to the object
    // collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "cobot_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::MoveGroupInterface gripper("hand");
    group.setPlanningTime(45.0);

    addCollisionObject(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    pre_pick(group);
    ros::WallDuration(1.0).sleep();
    pick(group);
    ros::WallDuration(1.0).sleep();
    close_gripper(gripper);
    ros::WallDuration(1.0).sleep();
    place(group);
    ros::WallDuration(1.0).sleep();
    open_gripper(gripper);


    ros::waitForShutdown();
    return 0;


}
