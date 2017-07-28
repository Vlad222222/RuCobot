#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
//#include <robot_state_publisher/robot_state_publisher.h>

#include <pluginlib/pluginlib_exceptions.h>
#include <pluginlib/class_loader.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_visual_tools/imarker_end_effector.h>
#include <moveit_visual_tools/imarker_robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_state/conversions.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/tf_visual_tools.h>

#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"


//#include <tf2_eigen/tf2_eigen.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>

#include <eigen_conversions/eigen_msg.h>

#include <QTimer>
#include <QElapsedTimer>
#include <QTime>

using namespace std;
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools* visual_tools;

struct Planner
{
    ros::NodeHandle node_handle;
    ros::ServiceClient client_get_scene_;

    moveit::planning_interface::MoveGroupInterface *move_group1, *move_group2;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene::PlanningScenePtr planning_scene;

    robot_model_loader::RobotModelLoader *robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;

    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup *joint_model_group1, *joint_model_group2;

    std::vector<std::string> joint_names1, joint_names2, link_names1, link_names2;

    //boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    //planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    string packPath, planGroupName1, planGroupName2;

    //moveit_visual_tools::MoveItVisualTools* visual_tools;

    //new
    planning_scene_monitor::PlanningSceneMonitor *planning_scene_monitor;

    Planner(string planGroup1,string planGroup2)
    {
        planGroupName1 = planGroup1;
        planGroupName2 = planGroup2;
        packPath = ros::package::getPath("moveit")+"/";

        move_group1 = new moveit::planning_interface::MoveGroupInterface(planGroupName1);
        move_group2 = new moveit::planning_interface::MoveGroupInterface(planGroupName2);
        robot_model_loader = new robot_model_loader::RobotModelLoader("robot_description");

        //new
        planning_scene_monitor = new planning_scene_monitor::PlanningSceneMonitor("robot_description");
        //  robot_model_loader = planning_scene_monitor->getRobotModel();
        //kinematic_model = planning_scene_monitor->getRobotModel();
        planning_scene = planning_scene_monitor->getPlanningScene();
        planning_scene_monitor->updatesScene(planning_scene);


        kinematic_model = robot_model_loader->getModel();
        kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
        joint_model_group1 = kinematic_model->getJointModelGroup(planGroupName1);
        joint_model_group2 = kinematic_model->getJointModelGroup(planGroupName2);

        //  move_group1->ge



        joint_names1 = joint_model_group1->getJointModelNames();
        joint_names2 = joint_model_group2->getJointModelNames();
        link_names1 = joint_model_group1->getLinkModelNames();
        link_names2 = joint_model_group2->getLinkModelNames();





        //planning_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(kinematic_model));

        //visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");

        //loadPlanner();
    }
    ~Planner()
    {

    }

    vector <double> getJointValues()
    {
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group1, joint_values);

        for (std::size_t i = 0; i < joint_names1.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names1[i].c_str(), joint_values[i]);
        }

        return joint_values;
    }

    bool setRandomPos()
    {
        robot_state::RobotState& current_state = planning_scene->getCurrentStateNonConst();
        current_state.setToRandomPositions(joint_model_group1);
        planning_scene->setCurrentState(current_state);
        sleep(1);
    }

    bool moveRandomPos(u_int armNumber)
    {
       // vector <double> curVal = getJointValues();

        moveit::planning_interface::MoveGroupInterface *move_group;
        robot_state::JointModelGroup *joint_model_group;

        switch (armNumber) {
        case 1:
            move_group = move_group1;
            joint_model_group = joint_model_group1;
            break;
        case 2:
            move_group = move_group2;
            joint_model_group = joint_model_group2;
            break;
        default:
            cout << "Bad armNumber" << endl;
            return false;
        }

        move_group->setStartStateToCurrentState();
        robot_state::RobotState goal_state = planning_scene->getCurrentState();

        for (u_int i=0; i<100; ++i)
        {
            goal_state.setToRandomPositions(joint_model_group);

            if (!isRobotStateCollision(goal_state))
            {
                break;
            }

            if (i==99)
            {
                cout << "Can't gen random pos" << endl;
                return false;
            }
        }

        Eigen::Affine3d end_effector_state = goal_state.getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());

        geometry_msgs::Pose idle_pose;
        tf::poseEigenToMsg(end_effector_state, idle_pose);
        move_group->setPoseTarget(idle_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group->setPlanningTime(10.0);

        if (move_group->plan(my_plan))
        {
            //move_group->execute(my_plan);

           /* ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
            moveit_msgs::DisplayTrajectory display_trajectory;

            ROS_INFO("Visualizing the trajectory");
          //  moveit_msgs::MotionPlanResponse response;
           // res.getMessage(response);

            display_trajectory.trajectory_start = my_plan.start_state_;
            display_trajectory.trajectory.push_back(my_plan.trajectory_);
            display_publisher.publish(display_trajectory);

sleep(3);*/

            sleep(3);

           // curVal = getJointValues();
            return true;
        }
        else
            return false;

    }




    geometry_msgs::PoseStamped getEndEffPose(string linkName)
    {
        robot_state::RobotState currentState = planning_scene->getCurrentState();

        const Eigen::Affine3d &tf = currentState.getGlobalLinkTransform(linkName);
        geometry_msgs::PoseStamped pose;
        tf::poseEigenToMsg(tf, pose.pose);
        cout << pose;
        return pose;
    }

    /*
    bool moveRandomPos1()
    {
        robot_state::RobotState start_state = planning_scene->getCurrentState();
        robot_state::RobotState goal_state = planning_scene->getCurrentStateNonConst();
        goal_state.setToRandomPositions(joint_model_group1);

        coutCurrentJoints();

        Eigen::Affine3d end_effector_state = goal_state.getGlobalLinkTransform(link_names1.back());

        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;

        planning_scene->checkCollision(collision_request, collision_result);
        if (!collision_result.collision)
            cout << endl<< "no collision" << endl<< endl;
        else
        {
            cout << endl<< "is collision" << endl<< endl;
            return false;
        }

        geometry_msgs::PoseStamped pose;
        tf::poseEigenToMsg(end_effector_state, pose.pose);

        pose.header.frame_id = "base_link";

        std::vector<double> tolerance_pose(3, 0.01);
        std::vector<double> tolerance_angle(3, 0.01);

        req.group_name = planGroupName1;
        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(link_names1.back(), pose, tolerance_pose, tolerance_angle);
        //        moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(link_names.back(), pose, tolerance_pose, tolerance_angle);
        req.goal_constraints.push_back(pose_goal);

        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
        //context->
        context->solve(res);
        if (res.error_code_.val != res.error_code_.SUCCESS)
        {
            ROS_ERROR("Could not compute plan successfully");
            return false;
        }

        cout << "VLAD ADVERTISE" << endl;
        ros::Publisher display_publisher =
                node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        moveit_msgs::DisplayTrajectory display_trajectory;

        ROS_INFO("Visualizing the trajectory");
        moveit_msgs::MotionPlanResponse response;
        res.getMessage(response);

        display_trajectory.trajectory_start = response.trajectory_start;
        display_trajectory.trajectory.push_back(response.trajectory);
        display_publisher.publish(display_trajectory);

        sleep(10);

        planning_scene->setCurrentState(goal_state);

        return true;
    }
    */

    void moveConstOrient()
    {

    }

    void moveCartesian()
    {
        // robot_state::RobotState goal_state = planning_scene->getCurrentStateNonConst();
        //move_group->setEndEffectorLink("ladon_l");
        //cout << planning_scene->getCurrentState()

        //move_group->getCurrentPose();
        cout << "Move Cartesian" << endl;

        geometry_msgs::PoseStamped ee1 = getEndEffPose(link_names1.back());
        geometry_msgs::PoseStamped ee2 = getEndEffPose(link_names2.back());

        std::vector<geometry_msgs::Pose> waypoints1, waypoints2;
        waypoints1.push_back(ee1.pose);
        waypoints2.push_back(ee2.pose);

        //X - left
        //Y - back
        /*
        ee.pose.position.y += 0.05;
        waypoints.push_back(ee.pose);
        ee.pose.position.x += 0.1;
        waypoints.push_back(ee.pose);
        ee.pose.position.z += 0.2;
        waypoints.push_back(ee.pose);
        ee.pose.position.x -= 0.3;
        waypoints.push_back(ee.pose);
        ee.pose.position.z -= 0.2;
        waypoints.push_back(ee.pose);
        ee.pose.position.x += 0.2;
        waypoints.push_back(ee.pose);
        ee.pose.position.y -= 0.05;
        waypoints.push_back(ee.pose);
*/

        ee1.pose.position.x += 0.05;
        waypoints1.push_back(ee1.pose);
        ee2.pose.position.x += 0.05;
        waypoints2.push_back(ee2.pose);

        ee1.pose.position.z += 0.05;
        waypoints1.push_back(ee1.pose);
        ee2.pose.position.z += 0.05;
        waypoints2.push_back(ee2.pose);

        ee1.pose.position.x -= 0.05;
        waypoints1.push_back(ee1.pose);
        ee2.pose.position.x -= 0.05;
        waypoints2.push_back(ee2.pose);

        ee1.pose.position.z -= 0.05;
        waypoints1.push_back(ee1.pose);
        ee2.pose.position.z -= 0.05;
        waypoints2.push_back(ee2.pose);



        // move_group.setMaxVelocityScalingFactor(0.1);


        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group1->computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory);
        double fraction2 = move_group2->computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory);



        //visual_tools->deleteAllMarkers();
        //visual_tools->publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools->publishPath(waypoints1, rvt::LIME_GREEN, rvt::SMALL);
        for (std::size_t i = 0; i < waypoints1.size(); ++i)
            visual_tools->publishAxisLabeled(waypoints1[i], "pt" + std::to_string(i), rvt::SMALL);

        visual_tools->publishPath(waypoints2, rvt::LIME_GREEN, rvt::SMALL);
        for (std::size_t i = 0; i < waypoints2.size(); ++i)
            visual_tools->publishAxisLabeled(waypoints2[i], "pt" + std::to_string(i), rvt::SMALL);

        visual_tools->trigger();

        //visual_tools->prompt("next step");
    }

    /*
    void loadPlanner()
    {
        planner_plugin_name = "ompl_interface/OMPLPlanner";
        if (!node_handle.getParam("planning_plugin", planner_plugin_name))
            ROS_FATAL_STREAM("Could not find planner plugin name");
        try
        {
            planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                                            "moveit_core", "planning_interface::PlannerManager"));
        }
        catch (pluginlib::PluginlibException& ex)
        {
            ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
        }
        try
        {
            planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
            if (!planner_instance->initialize(kinematic_model, node_handle.getNamespace()))
                ROS_FATAL_STREAM("Could not initialize planner instance");
            ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
        }
        catch (pluginlib::PluginlibException& ex)
        {
            const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
            std::stringstream ss;
            for (std::size_t i = 0; i < classes.size(); ++i)
                ss << classes[i] << " ";
            ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                             << "Available plugins: " << ss.str());
        }
    }
    */


    /*
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

    Now, we can print out the IK solution (if found):

    if (found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }*/

    bool checkSelfCollision(planning_scene::PlanningScene* planning_scene)
    {
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        planning_scene->checkSelfCollision(collision_request, collision_result);
        return collision_result.collision;

    }

    bool checkRightArmCollision(planning_scene::PlanningScene* planning_scene)
    {
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.group_name = "right_arm";
        planning_scene->checkSelfCollision(collision_request, collision_result);
        return collision_result.collision;
    }


    bool checkAllCollision()
    {
        planning_scene::PlanningScene planning_scene(kinematic_model);

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

        robot_state::RobotState copied_state = planning_scene.getCurrentState();
        const robot_state::JointModelGroup *joint_model_group1 = kinematic_model->getJointModelGroup(planGroupName1);

        std::vector<double> joint_values;
        copied_state.copyJointGroupPositions(joint_model_group1, joint_values);

        for (std::size_t i = 0; i < joint_values.size(); ++i)
        {
            cout << joint_values[i] << " ";
        }
        cout << endl;

        planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
        ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "IN" : "not in") << " collision");
        return collision_result.collision;
    }

    bool isRobotStateCollision(robot_state::RobotState& robSt)
    {
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        //   collision_detection::CollisionRobotConstPtr colRob = planning_scene->getCollisionRobot();












        /*

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = move_group1->getPlanningFrame();
        collision_object.id = "box2";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;

        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1;
        primitive.dimensions[1] = 0.4;
        primitive.dimensions[2] = 0.2;

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.09;
        box_pose.position.y = -0.3;
        box_pose.position.z = 0.33;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject>collision_objects;
        collision_objects.push_back(collision_object);
        ROS_INFO("Add an object to the world");

        planning_scene_interface.addCollisionObjects(collision_objects);*/

        //planning_scene_interface.applyCollisionObjects(collision_objects);

        //planning_scene->setPlanningSceneDiffMsg();











        collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
         // acm.print(cout);

        // acm.setEntry("box", false);

        /*
 ros::Publisher m_planningScenePub = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1, true);
*//*
moveit_msgs::AllowedCollisionEntry eValues;

acm.entry_names.push_back("box");
    eValues.enabled.push_back(false);
    acm.entry_values.push_back(eValues);*/


        //acm.print(cout);


        /*
810                 // Edit collision matrix;
811
812                 if(acm.entry_names.size()==0)
813                 {
814                         acm.entry_names.push_back(object.id);
815                         eValues.enabled.push_back(true);
816                         for(int i=0; i<m_touchLinks.size(); i++)
817                         {
818                                 acm.entry_names.push_back(m_touchLinks[i]);
819                                 eValues.enabled.push_back(true);
820                         }
821                         for(int e=0; e<m_touchLinks.size()+1; e++)
822                                 acm.entry_values.push_back(eValues);
823                 }
824                 else
825                 {
826
827                         acm.entry_names.push_back(object.id);
828                         ROS_INFO("Entries in Allowed Collision matrix %d", int(acm.entry_names.size()));
829
830                         for(int i=0; i<acm.entry_names.size()-1; i++)
831                         {
832                                 bool condition = false;
833                                 for(int j=0; j<m_touchLinks.size(); j++)
834                                 {
835                                         condition=condition||acm.entry_names[i]==m_touchLinks[j];
836                                         if(condition)
837                                                 break;
838                                 }
839                                 if(condition)
840                                 {
841                                         acm.entry_values[i].enabled.push_back(true);
842                                         eValues.enabled.push_back(true);
843                                         atObj.touch_links.push_back(acm.entry_names[i]); // prepare attached object!!! for later
844                                 }else
845                                 {
846                                         acm.entry_values[i].enabled.push_back(false);
847                                         eValues.enabled.push_back(false);
848                                 }
849                         }
850                         eValues.enabled.push_back(false);
851                         acm.entry_values.push_back(eValues);
852                 }
853                 scene.allowed_collision_matrix = acm;
854         }
*/











        /* acmMsg.setDefaultEntry("box", false);
        acm.setEntry("box", false);*/
        /*  acm.print(cout);
        exit(0);*/
        //  moveit_msgs::ApplyPlanningScene applyPlSc;
        //applyPlSc.
        //ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::ApplyPlanningScene>("applyPlSc");
        //planning_scene_diff_publisher.publish(applyPlSc);
        //planning_scene->set
        planning_scene->checkCollision(req, res, robSt, acm);


        /*   std::vector<std::string> names1;
        acm.getAllEntryNames(names1);

        for (u_int i=0; i<names1.size(); ++i)
            cout << names1[i] << endl;

        acm.*/

        //      exit(0);
        /*   collision_detection::CollisionResult::ContactMap& contacts;
                                 const robot_state::RobotState& kstate,
                                 const collision_detection::AllowedCollisionMatrix& acm
        planning_scene->getCollidingPairs();*/

        //planning_scene->isStateColliding(checkCollision(req, res, robSt, acm);

        // for (u_int i=0; i<res.ContactMap.size(); ++i)

        //cout << "contacts " << res.contact_count << res. << endl;
        return res.collision;
    }

    void coutCurrentJoints()
    {
        cout << "Current joint values" << endl;
        // const robot_state::JointModelGroup* joint_model_group666 = move_group->getCurrentState()->getJointModelGroup(planGroupName);
        //  vector <double> current_state = move_group->getCurrentJointValues();

        robot_state::RobotState cur_state = planning_scene->getCurrentState();
        Eigen::Affine3d end_effector_state = cur_state.getGlobalLinkTransform(link_names1.back());
        cout << end_effector_state.matrix() << endl;
        //  current_state->copyJointGroupPositions(joint_model_group666, joint_group_positions);

        /* for (std::size_t i = 0; i < current_state.size(); ++i)
        {
            cout << current_state[i] << " ";
        }*/
        cout << endl;
    }

    void printInfo()
    {
        cout << "Active joints" << endl;
        vector <string> vec = move_group1->getActiveJoints();
        for (int i=0; i< vec.size(); ++i)
            cout << vec[i] << endl << endl;

        cout << "LINKS" << endl;
        vec = move_group1->getRobotModel()->getJointModelNames();
        vec = move_group1->getRobotModel()->getLinkModelNames();
        for (int i=0; i< vec.size(); ++i)
            cout << vec[i] << endl;

        cout << "DefaultPlannerId" << endl;
        cout << move_group1->getDefaultPlannerId(planGroupName1) << endl;

        vec = move_group1->getNamedTargets();
        for (int i=0; i< vec.size(); ++i)
            cout << vec[i] << endl;
    }






};

#endif
