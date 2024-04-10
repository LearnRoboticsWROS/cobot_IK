#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// tau = 1 rotation in radiants
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  
    posture.joint_names.resize(2);
    posture.joint_names[0] = "gripper_right_joint";
    posture.joint_names[1] = "gripper_left_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0;
    posture.points[0].positions[1] = 0;
    posture.points[0].time_from_start = ros::Duration(0.5);


}


void closedGripper(trajectory_msgs::JointTrajectory& posture)
{

    posture.joint_names.resize(2);
    posture.joint_names[0] = "gripper_right_joint";
    posture.joint_names[1] = "gripper_left_joint";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.055;
    posture.points[0].positions[1] = -0.055;
    posture.points[0].time_from_start = ros::Duration(0.5);

    


}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;

    // Pick test 1
    /*
    orientation.setRPY(-tau/4, - tau/4, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 1;
    grasps[0].grasp_pose.pose.position.y = -0.085;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // The grasp will move in y direction
    grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Now the object is picked, we need to retreat in a certain direction the end effector
    // Let's retreat the end effecto upwards: positive z
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;
    */

    // Pick Test 2
    orientation.setRPY(tau/4, -tau/4, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 1;
    grasps[0].grasp_pose.pose.position.y = 0.085;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // Direction is set as positive x axis
    grasps[0].pre_grasp_approach.direction.vector.y = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    //Direction is set as positive z axis 
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.05;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;



    // we need to open the gripper. We will define a function for that
    openGripper(grasps[0].pre_grasp_posture);

    // When it grasps it needs to close the gripper
    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table 1
    move_group.setSupportSurfaceName("table1");

    // Call pick to pick up the object using the grasps given
    move_group.pick("object", grasps);

    
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;

    // Test 1
    /*
    orientation.setRPY(0, 0, tau/4); // A quarter turn about the z-axis
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 1;
    place_location[0].place_pose.pose.position.z = 0.5;

    // Setting pre place approach
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    // To pose in negative z direction
    place_location[0].pre_place_approach.direction.vector.z = -1;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    // To retreat in positive x
    place_location[0].post_place_retreat.direction.vector.x = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;
    */
    // Test 2
    orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 1;
    //place_location[0].place_pose.pose.position.z = 0.5;
    place_location[0].place_pose.pose.position.z = 0.6;

    // Setting pre-place approach
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    // Direction is set as negative z axis
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    

    // Setting post-grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    // Direction is set as negative y axis
    place_location[0].post_place_retreat.direction.vector.x = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    openGripper(place_location[0].post_place_posture);

    // Set support surface as table 2
    group.setSupportSurfaceName("table2");
    
    // Call place to palce the object using the place location given
    group.place("object", place_location);


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

    // Add the object to be picked
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";

    // Define primitive dimension, position of the object
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;
    // pose of object
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 1;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the object
    collision_objects[2].operation = collision_objects[2].ADD;

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
    group.setPlanningTime(45.0);

    // Put the object in the scene
    addCollisionObject(planning_scene_interface);

    // Wait for initialization
    ros::WallDuration(1.0).sleep();

    // Pick the object
    pick(group);

    ros::WallDuration(1.0).sleep();

    // Place the object
    place(group);

    ros::waitForShutdown();
    return 0;

}