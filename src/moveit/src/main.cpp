#include <planner.h>



bool checkSelfCollision(planning_scene::PlanningScene*);
bool checkAllCollision(robot_model::RobotModelPtr);
bool addSTL(Planner &);
bool addBox(Planner &);
void publishBox(Planner&);


/*int main( int argc, char** argv )
{
    ros::init(argc, argv, "rw_move_group_interface_test", ros::init_options::AnonymousName);

    ros::AsyncSpinner spinner(1);
    spinner.start();

     moveit::planning_interface::MoveGroupInterface group("left_arm");

    std::vector<double> v;

    v.push_back(1.15);
    v.push_back(-0.105);
    v.push_back(0.38);
    v.push_back(0.055);
    v.push_back(-1.47);
    v.push_back(0);
    v.push_back(0);

    const string name = "ladon_l";

    group.setJointValueTarget(v, name);
    group.move();
    sleep(2);

    std::vector<double> q;
    q.push_back(-0.69);
    q.push_back(0.52);
    q.push_back(1.09);
    q.push_back(0.8);
    q.push_back(0.23);
    q.push_back(0);
    q.push_back(0);

    group.setJointValueTarget(q);
    group.move();
    sleep(1);



  return 0;
}*/


int main( int argc, char** argv )
{
    ros::init(argc, argv, "moveit");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");

    Planner planner("left_arm", "right_arm");

    planner.client_get_scene_= planner.node_handle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    //planner.printInfo();


    //  Planner planner2("right_arm");

    //moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

//        addBox(planner);
    publishBox(planner);

    for (int i=0; i< 1000; ++i)
    {
        ros::Time begin = ros::Time::now();
        if (planner.moveRandomPos(1))
        {
            cout << ros::Time::now()-begin << endl;
        }
        //

        //planner2.moveRandomPos();
    }

    // planner.moveCartesian();
    //sleep(5);








    ros::shutdown();
    return 0;
}





bool addBox(Planner& planner)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = planner.move_group1->getPlanningFrame();
    collision_object.id = "box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.050;
    primitive.dimensions[1] = 0.50;
    primitive.dimensions[2] = 2;

    geometry_msgs::Pose stlPose;
    stlPose.orientation.w = 1.0;
    stlPose.position.x = 0.3; //left
    stlPose.position.y = -0.2; //nazad
    stlPose.position.z = 1.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(stlPose);
    collision_object.operation = collision_object.ADD;

    planner.node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 0);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planner.planning_scene_interface.addCollisionObjects(collision_objects);

    ros::Duration(1.0).sleep();

    moveit_msgs::PlanningScene planning_scene;
    ros::Publisher planning_scene_diff_publisher = planner.node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    sleep(2.0);

    planner.planning_scene = planner.planning_scene_monitor->getPlanningScene();
    collision_detection::AllowedCollisionMatrix acm = planner.planning_scene->getAllowedCollisionMatrixNonConst();
    acm.print(cout);
    collision_detection::AllowedCollisionMatrix acm1 = planner.planning_scene->getAllowedCollisionMatrix();
    acm1.print(cout);


    //    moveit_msgs::AllowedCollisionEntry eValues;
    acm.setDefaultEntry("box", false);
    //    eValues.enabled.push_back(false);
    //    acm.entry_values.push_back(eValues);
    acm.print(cout);
    // planner.planning_scene->se
    // acm.getMessage();
    exit(0);
}







void publishBox(Planner& planner)
{
    geometry_msgs::Pose stlPose;
    stlPose.orientation.w = 1.0;
    stlPose.position.x = 0.3; //left
    stlPose.position.y = -0.2; //nazad
    stlPose.position.z = 1.5;

  /*  geometry_msgs::PoseStamped handle_pose;
    handle_pose.pose.orientation.w = 1.0;
    handle_pose.pose.position.x = 0.3; //left
    handle_pose.pose.position.y = -0.2; //nazad
    handle_pose.pose.position.z = 1.5;*/

    //publish collision object
    moveit_msgs::CollisionObject object;
   // object.header = handle_pose.header;
    object.header.frame_id = planner.move_group1->getPlanningFrame();
    object.id = "box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.050;
    primitive.dimensions[1] = 0.50;
    primitive.dimensions[2] = 2;
    /*
        primitive.dimensions[0] = 1000;
        primitive.dimensions[1] = 800;
        primitive.dimensions[2] = 2000;
        */

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(stlPose);
    object.operation = object.ADD;













    planner.node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 0);
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(object);
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planner.planning_scene_interface.addCollisionObjects(collision_objects);
sleep(2.0);


    moveit_msgs::PlanningScene planning_scene_add;
    planning_scene_add.world.collision_objects.push_back(object);
    planning_scene_add.is_diff = true;

    ros::Publisher planning_scene_diff_publisher = planner.node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    planning_scene_diff_publisher.publish(planning_scene_add);
    sleep(2.0);

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;


    //wait until box is published (I don't think that this is really needed)
    bool object_in_world = false;

    while(!object_in_world)
    {
        ROS_ERROR("waiting for box to appear");

        if (planner.client_get_scene_.call(srv))
        {
            for (int i = 0; i < (int)srv.response.scene.world.collision_objects.size(); ++i)
            {
                if (srv.response.scene.world.collision_objects[i].id == object.id)
                    object_in_world = true;
            }
        }
    }


    moveit_msgs::PlanningScene currentScene;
    moveit_msgs::PlanningScene newSceneDiff;
    moveit_msgs::GetPlanningScene scene_srv;

    scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

    if(!planner.client_get_scene_.call(scene_srv))
    {
        ROS_WARN("Failed to call service /get_planning_scene");
    }
    else
    {
        ROS_INFO_STREAM("Initial scene!");
        currentScene = scene_srv.response.scene;
        moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

        ROS_ERROR_STREAM("size of acm_entry_names before " << currentACM.entry_names.size());
        ROS_ERROR_STREAM("size of acm_entry_values before " << currentACM.entry_values.size());
        ROS_ERROR_STREAM("size of acm_entry_values[0].entries before " << currentACM.entry_values[0].enabled.size());

        currentACM.entry_names.push_back(object.id);
        moveit_msgs::AllowedCollisionEntry entry;
        entry.enabled.resize(currentACM.entry_names.size());

        for(int i = 0; i < entry.enabled.size(); i++)
        entry.enabled[i] = false;

        //add new row to allowed collsion matrix
        currentACM.entry_values.push_back(entry);

        for(int i = 0; i < currentACM.entry_values.size(); i++)
        {
            //extend the last column of the matrix
            currentACM.entry_values[i].enabled.push_back(false);
        }

        newSceneDiff.is_diff = true;
        newSceneDiff.allowed_collision_matrix = currentACM;

        planning_scene_diff_publisher.publish(newSceneDiff);


    }

    if(!planner.client_get_scene_.call(scene_srv))
    {
        ROS_WARN("Failed to call service /get_planning_scene");
    }

    else
    {
        ROS_INFO_STREAM("Modified scene!");

        currentScene = scene_srv.response.scene;
        moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

        ROS_ERROR_STREAM("size of acm_entry_names after " << currentACM.entry_names.size());
        ROS_ERROR_STREAM("size of acm_entry_values after " << currentACM.entry_values.size());
        ROS_ERROR_STREAM("size of acm_entry_values[0].entries after " << currentACM.entry_values[0].enabled.size());
    }
}










bool addSTL(Planner& planner)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = planner.move_group1->getPlanningFrame();
    collision_object.id = "scene";

    geometry_msgs::Pose stlPose;
    stlPose.orientation.w = 1.0;
    stlPose.position.x = 0.3; //left
    stlPose.position.y = -0.2; //nazad
    stlPose.position.z = 0;

    //shapes::Mesh* stlShape = shapes::createMeshFromResource("package://moveit/room.stl");
    //stlShape->scale(0.1);
    shapes::Mesh* stlShape = shapes::createMeshFromResource("package://moveit/Rock.dae");
    shape_msgs::Mesh stl_mesh;
    shapes::ShapeMsg stl_mesh_msg;
    shapes::constructMsgFromShape(stlShape, stl_mesh_msg);
    stl_mesh = boost::get<shape_msgs::Mesh>(stl_mesh_msg);

    collision_object.meshes.push_back(stl_mesh);
    collision_object.mesh_poses.push_back(stlPose);
    collision_object.operation = collision_object.ADD;
    planner.node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 0);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planner.planning_scene_interface.addCollisionObjects(collision_objects);

    ros::Duration(1.0).sleep();

    moveit_msgs::PlanningScene planning_scene;
    ros::Publisher planning_scene_diff_publisher = planner.node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    sleep(2.0);

    //cout << "GOOD " <<planner.planning_scene->processCollisionObjectMsg(collision_object) << endl;
    vector <string> id_names;
    id_names = planner.planning_scene_interface.getKnownObjectNames();
    cout << "size " << id_names.size() << endl;
    for(int i=0; i<id_names.size();++i)
        cout << id_names[i] << endl;
    //exit(0);
    /* moveit_msgs::PlanningScene scene1;
    planner.planning_scene->getPlanningSceneMsg(scene1);
    planner.planning_scene_interface.applyPlanningScene(scene1);*/

    planner.planning_scene = planner.planning_scene_monitor->getPlanningScene();


}

/*
void add_text()
{
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 2; // above head of PR2

    visual_tools->publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools->trigger();
}
*/

/*
void addCollisionObject()
 {
       moveit_msgs::CollisionObject object;
       object.id = "box";
     object.header = m_scene.robot_state.joint_state.header;
      moveit_msgs::PlanningScene scene;
       moveit_msgs::AttachedCollisionObject atObj;
       planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(m_model )  );
     planning_scene->getPlanningSceneMsg(scene);
      moveit_msgs::AllowedCollisionMatrix acm = scene.allowed_collision_matrix;
      scene.robot_state = m_scene.robot_state;
 777         scene.world.collision_objects = m_scene.world.collision_objects;
 778         if(m_base_link!="/base_link")
 779         {
 780                 try{
 781                 m_transformListener->waitForTransform("/base_link", m_base_link, ros::Time::now(), ros::Duration(0.5));
 782                 geometry_msgs::PoseStamped orig, trans;
 783                 orig.header.frame_id = m_base_link;
 784                 orig.pose = pose;
 785                 m_transformListener->transformPose("/base_link", orig, trans);
 786                 pose = trans.pose;
 787                                 }
 788
 790         }
 791         if(m_scene.world.collision_objects.size()==0)
 792         {
 793                 geometry_msgs::Pose objPose = pose;
 794                 objPose.position.z-=0.1;
 795                 objPose.orientation.x = objPose.orientation.y = objPose.orientation.z =0.0;
 796                 objPose.orientation.w = 1.0;
 797                 object.primitive_poses.push_back(objPose);
 798                 shape_msgs::SolidPrimitive shape;
 799                 shape.type = shape_msgs::SolidPrimitive::CYLINDER;
 800                 shape.dimensions.push_back(0.25);
 801                 shape.dimensions.push_back(0.1);
 802                 object.primitives.push_back(shape);
 803                 object.operation = moveit_msgs::CollisionObject::ADD;
 804
 805                 atObj.object = object;
 806                 atObj.object.id = "attachedObject";
 807                 atObj.object.operation = moveit_msgs::CollisionObject::ADD;
 808                 atObj.link_name = "object_link";
 809                 atObj.weight = 0.8;
 810                 // Edit collision matrix;
 811                 moveit_msgs::AllowedCollisionEntry eValues;
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
 855         else
 856                 object.operation = moveit_msgs::CollisionObject::MOVE;
 857
 858
 859         scene.world.collision_objects.push_back(object);
 860         scene.is_diff = true;
 861         m_planningScenePub.publish(scene);
 862         ros::spinOnce();
 863 }*/
