#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <signal.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// Subscriber msgs
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <jaco_msgs/JointVelocity.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"

// Own arm libraries
#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

#define NUM_JOINTS 8 //6+2 for the arm

// Define bounding box boundaries
double minX = .25;
double maxX = .5;
double minY = -.3;
double maxY = .3;

// TODO test joint locationt

//mico joint state safe
//-2.3321322971114142, -1.6372086401627464, -0.28393691436045176, -2.164605083475533, 0.7496982226688764, 4.682638807847723

/* tool pose side
    position: 
        x: 0.117240786552
        y: -0.301719456911
        z: 0.239926770329
      orientation: 
        x: 0.51289595084
        y: 0.484664185494
        z: 0.517808228151
        w: 0.483645541456
    tool pose safe
        
    x: -0.157769784331
    y: -0.136029005051
    z: 0.376786500216
  orientation: 
    x: 0.994340247286
    y: 0.0977247708014
    z: 0.005313327657
    w: 0.0413413878465
*/

// Joint vel publisher
ros::Publisher j_vel_pub_;
// Cartesian vel publisher
ros::Publisher c_vel_pub_;

//global variables for storing data
sensor_msgs::JointState current_state;
bool heardJoinstState;

geometry_msgs::PoseStamped current_pose;
geometry_msgs::Pose tool_pos_cur;
bool heardPose;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

// Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
    if (input->position.size() == NUM_JOINTS){
        current_state = *input;
        heardJoinstState = true;
    }
}

// Toolpos cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
    tool_pos_cur = msg.pose;
    current_pose = msg;
    heardPose = true;
}

// Blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
    heardJoinstState = false;
    heardPose = false;
    ros::Rate r(10.0);
    
    while (ros::ok()){
        ros::spinOnce();
        
        if (heardJoinstState && heardPose)
            return;
        
        r.sleep();
    }
}

// Blocking call for user input
void pressEnter(std::string message){
    std::cout << message;
    while (true){
        char c = std::cin.get();
        if (c == '\n')
            break;
        else if (c == 'q'){
            ros::shutdown();
            exit(1);
        }
        else {
            std::cout <<  message;
        }
    }
}

sensor_msgs::JointState getStateFromBag(std::string bagName){
    rosbag::Bag bag;
    std::string path = ros::package::getPath("segbot_arm_explore");
    
    bag.open(path + "/positions/" + bagName + ".bag", rosbag::bagmode::Read);
    
    rosbag::View view(bag, rosbag::TopicQuery("joint_states"));
    sensor_msgs::JointState fromFile;
    BOOST_FOREACH(rosbag::MessageInstance const m, view){
        sensor_msgs::JointState::ConstPtr traj = m.instantiate<sensor_msgs::JointState>();
        if (traj != NULL){
            fromFile = *traj;
        }
    }
    bag.close();
    
    /*actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);
    jaco_msgs::ArmJointAnglesGoal goal;
    goal.angles.joint1 = fromFile.position[0];
    goal.angles.joint2 = fromFile.position[1];
    goal.angles.joint3 = fromFile.position[2];
    goal.angles.joint4 = fromFile.position[3];
    goal.angles.joint5 = fromFile.position[4];
    goal.angles.joint6 = fromFile.position[5];
    //ROS_INFO("Joint6: %f", fromFile.position[5]);
    ac.waitForServer();
    ac.sendGoal(goal);
    ROS_INFO("Trajectory goal sent!");
    ac.waitForResult();
    */
    return fromFile;
}

void moveX(ros::NodeHandle n, double x){
    listenForArmData();
    
    geometry_msgs::PoseStamped p_target = current_pose;
    
    p_target.pose.position.x += x;
    segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
} 

void moveAway(ros::NodeHandle n, double x) {
    moveX(n, x);
}

void moveTowards(ros::NodeHandle n, double x) {
    moveX(n, -x);
}

void moveY(ros::NodeHandle n, double y) {
    listenForArmData();
    
    geometry_msgs::PoseStamped p_target = current_pose;
    
    p_target.pose.position.y += y;
    segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
}

void moveLeft(ros::NodeHandle n, double y) {
    moveY(n, y);
}

void moveRight(ros::NodeHandle n, double y) {
    moveY(n, -y);
}

void goToSafePose(ros::NodeHandle n){
    geometry_msgs::PoseStamped pose_st;
    pose_st.header.stamp = ros::Time(0);
    pose_st.header.frame_id = "mico_link_base";
}

bool goToLocation(sensor_msgs::JointState js){
    moveit_utils::AngularVelCtrl srv;
    srv.request.state = js;
    /*if(angular_client.call(srv))
        ROS_INFO("Sending angular commands");
    else
        ROS_INFO("Cannot contact angular velocity service. Is it running?");
    clearMsgs(.5);
    return srv.response.success;*/
    actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);
    jaco_msgs::ArmJointAnglesGoal goal;
    goal.angles.joint1 = js.position[0];
    goal.angles.joint2 = js.position[1];
    goal.angles.joint3 = js.position[2];
    goal.angles.joint4 = js.position[3];
    goal.angles.joint5 = js.position[4];
    goal.angles.joint6 = js.position[5];
    //ROS_INFO("Joint6: %f", fromFile.position[5]);
    ac.waitForServer();
    ac.sendGoal(goal);
    ROS_INFO("Trajectory goal sent");
    ac.waitForResult();
}

void lift(ros::NodeHandle n, double z) {
    listenForArmData();
    
    geometry_msgs::PoseStamped p_target = current_pose;
    
    p_target.pose.position.z += z;
    segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
} 

// Generate a random floating point number from min to max
double randfrom(double min, double max) 
{
    static bool first = true;
    if(first) {
        srand(time(NULL));
        first = false;
    } 

    double range = (max - min); 
    double div = RAND_MAX / range;
    return min + (rand() / div);
} 

// Returns random x to move within bounding box (+/-)
double randomX() {
    listenForArmData();
    double posX = current_pose.pose.position.x; 

    double minXMove = minX - posX;
    double maxXMove = maxX - posX;

    double rand = randfrom(minXMove, maxXMove);
    return rand;
}

// Returns random y to move within bounding box (+/-)
double randomY() {
    listenForArmData();
    double posY = current_pose.pose.position.y; 

    double minYMove = minY - posY;
    double maxYMove = maxY - posY;

    double rand = randfrom(minYMove, maxYMove);
    return rand;
}

void moveRandom(ros::NodeHandle n) {
    double randX = randomX();
    double randY = randomY();

    ROS_INFO("Moving %f x, %f y", randX, randY);
    moveX(n, randX);
    moveY(n, randY);
}

void moveToShakePos(ros::NodeHandle n) {
    std::string j_pos_filename = ros::package::getPath("segbot_arm_explore")+"/data/jointspace_position_db.txt";
    std::string c_pos_filename = ros::package::getPath("segbot_arm_explore")+"/data/toolspace_position_db.txt";
    
    ArmPositionDB *posDB = new ArmPositionDB(j_pos_filename, c_pos_filename);

    /*
    // Move to gemoetry pose (may twist joints into weird position) 
    if (posDB->hasCarteseanPosition("shake")) {
        ROS_INFO("Moving to shake starting position by geometry pose...");
        geometry_msgs::PoseStamped shake_pose = posDB->getToolPositionStamped("shake","/mico_link_base");
            
        // Now go to the pose
        segbot_arm_manipulation::moveToPoseMoveIt(n, shake_pose);
        segbot_arm_manipulation::moveToPoseMoveIt(n, shake_pose);

    } else {
        ROS_ERROR("[demo_shake_explore.cpp] Cannot move arm to shaking position!");
    }
    */

    // Move to joint pose
    /*if(posDB->hasJointPosition("shake")) {
        ROS_INFO("Moving to shake starting position by joint pose...");
        std::vector<float> shake_pos = posDB->getJointPosition("shake");
        sensor_msgs::JointState joint_msg;
        for(int i = 1; i <= 6; i = i + 1) {
            joint_msg.position.push_back(shake_pos[i]);
        }
        goToLocation(joint_msg);
    } else {
        ROS_ERROR("[demo_shake_explore.cpp] Cannot move arm to shaking position!");
    }
    */

    // Move to ros bog position
    ROS_INFO("Moving to shake staring position by ros bag position...");
    sensor_msgs::JointState loc = getStateFromBag("drop_right");
    goToLocation(loc);   
}

bool shake(double vel) {
    int iterations = 2;
    int count = 0;
    double step = .25;
    double distance = 40; //degrees
    if(vel > 1)
        vel = 1;
    jaco_msgs::JointVelocity T;
    ros::Rate r(4);
    T.joint1 = 0.0;
    T.joint2 = 0.0;
    T.joint3 = 0.0;
    T.joint4 = 0.0;
    T.joint5 = 0.0;
    T.joint6 = 0.0;

    vel *= 180/3.1459;
    int sign = 1;
    double tempDistance;
    bool firstOrLast = true;
    while(count < iterations){
        for(int i = 0; i < distance/vel/step; i++){
            ROS_INFO("Got vel: %f",vel);
            
            T.joint3 = vel;
            T.joint4 = vel;
            T.joint5 = vel;
            T.joint6 = 4*vel;
            
            j_vel_pub_.publish(T);
            r.sleep();

        }
        T.joint4 = 0.0;
        T.joint5 = 0.0;
        T.joint6 = 0.0;
        
        j_vel_pub_.publish(T);
        for(int i = 0; i < distance/vel/step; i++){
            ROS_INFO("Got vel: %f",vel);
            
            T.joint3 = -vel;
            T.joint4 = -vel;
            T.joint5 = -vel;
            T.joint6 = 4*vel;
            
            j_vel_pub_.publish(T);
            r.sleep();
        }
        T.joint3 = 0.0;
        T.joint4 = 0.0;
        T.joint5 = 0.0;
        T.joint6 = 0.0;
        
        j_vel_pub_.publish(T);
        for(int i = 0; i < distance/vel/step; i++){
            ROS_INFO("Got vel: %f",vel);
            T.joint3 = -vel;
            T.joint4 = -vel;
            T.joint5 = -vel;
            T.joint6 = -4*vel;
            
            j_vel_pub_.publish(T);
            r.sleep();
        }
        T.joint3 = 0.0;
        T.joint4 = 0.0;
        T.joint5 = 0.0;
        T.joint6 = 0.0;
        
        j_vel_pub_.publish(T);
        for(int i = 0; i < distance/vel/step; i++){
            ROS_INFO("Got vel: %f",vel);
            T.joint3 = vel;
            T.joint4 = vel;
            T.joint5 = vel;
            T.joint6 = -4*vel;
            
            j_vel_pub_.publish(T);
            r.sleep();
        }
        T.joint3 = 0.0;
        T.joint4 = 0.0;
        T.joint5 = 0.0;
        T.joint6 = 0.0;
        
        j_vel_pub_.publish(T);
        count++;
    }
    T.joint3 = 0.0;
    T.joint4 = 0.0;
    T.joint5 = 0.0;
    T.joint6 = 0.0;
    
    j_vel_pub_.publish(T);
}

//TODO Find and move to best position to press object 
void moveFindPressPos() {
    
}

bool press(double velocity) {
    // Move arm to slightly above object (ideally centered)
    moveFindPressPos();

    ros::Rate r(40);
    geometry_msgs::TwistStamped T;
        T.twist.linear.x= 0.0;
    T.twist.linear.y= 0.0;
    T.twist.linear.z= 0.0;
    T.twist.angular.x= 0.0;
    T.twist.angular.y= 0.0;
    T.twist.angular.z= 0.0;
    
    geometry_msgs::Pose tool_pose_last = tool_pos_cur;
    while(tool_pos_cur.position.z <= tool_pose_last.position.z){
        T.twist.linear.z = -velocity;
        c_vel_pub_.publish(T);
        r.sleep();
        tool_pose_last = tool_pos_cur;
        ros::spinOnce();
    } 
    T.twist.linear.z= 0.0;
    c_vel_pub_.publish(T);
}


int main(int argc, char **argv) {
    // Intialize ROS with this node name
    ros::init(argc, argv, "demo_grasp_explore");
    
    ros::NodeHandle n;

    //create subscriber to joint angles
    ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
    
    //create subscriber to tool position topic
    ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

    // Publisher for cartesian velocity
    c_vel_pub_ = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);
    j_vel_pub_ = n.advertise<jaco_msgs::JointVelocity>("/mico_arm_driver/in/joint_velocity", 2);

    //register ctrl-c
    signal(SIGINT, sig_handler);
    
    //store out-of-view position here
    sensor_msgs::JointState joint_state_outofview;
    geometry_msgs::PoseStamped pose_outofview;

    pressEnter("Press enter: demo starting...move the arm to a position where it is not occluding the table.");
    
    //store out of table joint position
    listenForArmData();
    joint_state_outofview = current_state;
    pose_outofview = current_pose;
    

    while (ros::ok()){
        //move the arm to side view
    
        //get the table scene
        segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);
        
        if ((int)table_scene.cloud_clusters.size() == 0){
            ROS_WARN("No objects found on table. The end...");
            exit(1);
        }
        
        //select the object with most points as the target object
        int largest_pc_index = -1;
        int largest_num_points = -1;
        for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
            
            int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
        
            if (num_points_i > largest_num_points){
                largest_num_points = num_points_i;
                largest_pc_index = i;
            }
        }
        
        //create the action client
        actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac("segbot_tabletop_grasp_as",true);
        ac.waitForServer();
        
        //create and fill goal
        segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
        
        //we want the robot to execute the GRASP action
        grasp_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::GRASP;
        
        //for that action, we have to specify the method used for picking the target grasp out of the candidates
        //grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
        grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_JOINTSPACE_SELECTION;
        
        //grasp_goal.grasp_filter_method=segbot_arm_manipulation::TabletopGraspGoal::TOPDOWN_GRASP_FILTER;
        //grasp_goal.grasp_filter_method=segbot_arm_manipulation::TabletopGraspGoal::SIDEWAY_GRASP_FILTER;
        
        //finally, we fill in the table scene
        grasp_goal.cloud_plane = table_scene.cloud_plane;
        grasp_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
        for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
            grasp_goal.cloud_clusters.push_back(table_scene.cloud_clusters[i]);
        }
        grasp_goal.target_object_cluster_index = largest_pc_index;
                
        //send the goal
        ROS_INFO("Sending grasp goal to action server...");
        ac.sendGoal(grasp_goal);
        
        //block until the action is completed
        ROS_INFO("Waiting for result...");
        
        bool result = ac.waitForResult();

        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Grasping action finished: %s",state.toString().c_str());
        if (state.state_ == actionlib::SimpleClientGoalState::ABORTED){\
            ROS_WARN("Grasping server aborted action...");
        }
        else {
            //lift and lower the object a bit, let it go and move back
            lift(n, 0.3);
            ROS_INFO("Lifting object");

            // Shake object
            //moveToShakePos(n);
            //shake(1.5);

            // Explore: Move object to new location
            ROS_INFO("Moving to random location");
            moveRandom(n);

            // Drop the object in its new location     
            ROS_INFO("Dropping object");     
            lift(n, -.3);
            segbot_arm_manipulation::openHand();

            // Home the arm
            ROS_INFO("Homing arm");
            segbot_arm_manipulation::homeArm(n);
            
            // Move out of view and try again
            ROS_INFO("Moving to out of view position");
            segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
        }
        
        /*sleep(2.0);
        lift(n,-0.07);
        segbot_arm_manipulation::openHand();
        lift(n,0.07);
        
        segbot_arm_manipulation::homeArm(n);
        segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
        */
    
        //return 1;
        pressEnter("Press 'Enter' to grasp again or 'q' to quit.");
    }
}
