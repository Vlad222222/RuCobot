#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <ros/timer.h>
#include <ros/time.h>
#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <gazebo/common/common.hh>

//#include "gazebo/common/URI.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

#include <sdf/sdf.hh>
#include <gazebo/physics/Base.hh>
//#include <gazebo/plugins/TimerGUIPlugin.hh>


using namespace std;

/*struct PID{
    float Kp{0}, Ki{0}, Kd{0};
};*/

void timerCallback(const ros::TimerEvent& ev)
{
    ROS_INFO("HI");
    cout << "Hi!" << endl;
}



namespace gazebo
{
math::Pose initPose;
float j1=0;
float j1_step = 1;

vector <physics::JointPtr> fixedJoints;

//ros::NodeHandlePtr nhptr;
// ros::NodeHandle* nhptr;
/// \brief A plugin to control a Velodyne sensor.
class VelodynePlugin : public ModelPlugin
{
    /// \brief A node used for transport
private: transport::NodePtr node;

    //private: ros::NodeHandle nhptr;
private: ros::NodeHandlePtr nh;

    //private:     ros::Timer timer;
    /// \brief A subscriber to a named topic.
private: transport::SubscriberPtr sub;

    /// \brief Pointer to the model.
private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
private: physics::JointPtr *joint;
private: physics::LinkPtr *link;

    /// \brief A PID controller for the joint.
public: vector <common::PID> pidBest;
    //    gazebo::common::Timer gz_timer;
private: event::ConnectionPtr ev;

    /// \brief Constructor
public: VelodynePlugin()
    {

    }
public: ~VelodynePlugin()
    {
        delete joint;
    }

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        //model-

        //        nhptr = boost::make_shared<ros::NodeHandle>();
        const string f = "gazebo_ros_control";
        //         nh = ros::NodeHandlePtr(f.c_str());
        //ros::NodeHandle n;
        //this->ev = event::Events::ConnectDiagTimerStart

        nh = boost::make_shared<ros::NodeHandle>();
        //  nh->

        model = _model;

        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cout << "Invalid joint count, Velodyne plugin not loaded\n";
            return;
        }
        else
        {
            std::cout << "Model has " << model->GetJointCount() <<  " joints\n";
            // std::cout << "Model has " << model->getGetGetJointCount() <<  " joints\n";
            //return;
        }

        joint = new physics::JointPtr[model->GetJointCount()];
        link = new physics::LinkPtr[model->GetJointCount()];

        pidBest.resize(model->GetJointCount());
        pidBest[3] = common::PID(4800, 8135.59, 708);
        pidBest[4] = common::PID(6000, 22641.5, 397.5);
        pidBest[5] = common::PID(720, 2666.67, 48.6);
        pidBest[6] = common::PID(720, 1920, 67.5);
        pidBest[7] = common::PID(84, 1200, 1.47);

        //joint[i]->g

        for(int i=1; i<model->GetJointCount(); ++i)
        {
            joint[i] =_model->GetJoints()[i];

            // joint[i]->
            /*    cout << "Joint " << i << " is " << joint[i]->GetScopedName() << endl;

            if (i!=5)
            {
                joint[i]->SetUpperLimit(0, 0);
                joint[i]->SetLowerLimit(0, 0);
                joint[i]->SetParam("friction", 0, 99999999999);
                joint[i]->SetParam("hi_stop", 0, 0);
                joint[i]->SetParam("lo_stop", 0, 0);
                joint[i]->Update();
            }*/
        }

        FixAllJoints();

        /*  joint[5] =_model->GetJoints()[5];
        joint[5]->SetUpperLimit(0, 1000);
        joint[5]->SetLowerLimit(0, -1000);
        joint[5]->SetParam("friction", 0, 0);
        joint[5]->Update();*/

        /* while (1)
        {
            cout << joint[5]->Position() << endl;
            cout << joint[5]->GetAngle(0) << endl;

            usleep(10000);
        }*/
        /*   joint[5] =_model->GetJoints()[5];
        joint[5]->SetUpperLimit(0, 1000);
        joint[5]->SetLowerLimit(0, -1000);
        joint[5]->SetParam("friction", 0, 0);
        joint[5]->Update();*/


        // Default to zero velocity
        double velocity = 0;

        velocity=0;

        //  this->SetJointPosition(0);


        // this->node->

        // Create the node
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(model->GetWorld()->GetName());

        // Create a topic name
        std::string topicName = "~/" + this->model->GetName() + "/joint";
        std::cout << "Model name " << this->model->GetName() << "\n";

        // Subscribe to the topic, and register a callback
        // this->sub = this->node->Subscribe(topicName, &VelodynePlugin::OnMsg, this);

        //ros::Timer timer = nhptr->createTimer(ros::Duration(0.05), timerCallback);

        // ros::Rate loop_rate(10); // 10 Hz

        // ros::NodeHandlePtr nh1("gazebo_ros_control");
        cout << "Callback" << endl;
        //        ros::Timer timer1 = nh->createTimer(ros::Duration(1), &timerCallback, &this);
        //timer1.start();


        //ros::spinOnce();


        boost::thread t(boost::bind( &gazebo::VelodynePlugin::asyncThread, this));  //   &gazebo::VelodynePlugin::asyncThread);

        //boost::thread t1(&thread2);

    }

public: void MovePath()
    {
        float vel = 5;

        joint[3]->SetVelocityLimit(0, vel);
        joint[4]->SetVelocityLimit(0, vel);
        joint[5]->SetVelocityLimit(0, vel);
        joint[6]->SetVelocityLimit(0, vel);
        joint[7]->SetVelocityLimit(0, vel);

        joint[3]->SetEffortLimit(0, 80);
        joint[4]->SetEffortLimit(0, 80);
        joint[5]->SetEffortLimit(0, 20);
        joint[6]->SetEffortLimit(0, 20);
        joint[7]->SetEffortLimit(0, 20);

        int damp = 0.7;
        joint[3]->SetDamping(0, damp);
        joint[4]->SetDamping(0, damp);
        joint[5]->SetDamping(0, damp);
        joint[6]->SetDamping(0, damp);
        joint[7]->SetDamping(0, damp);

        UnfixJoint(3); UnfixJoint(4); UnfixJoint(5); UnfixJoint(6); UnfixJoint(7);

        model->GetJointController()->SetPositionPID(joint[3]->GetScopedName(), pidBest[3]);
        model->GetJointController()->SetPositionPID(joint[4]->GetScopedName(), pidBest[4]);
        model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), pidBest[5]);
        model->GetJointController()->SetPositionPID(joint[6]->GetScopedName(), pidBest[6]);
        model->GetJointController()->SetPositionPID(joint[7]->GetScopedName(), pidBest[7]);


        MovePoint (0,0,0,0,0);

        float ang = 10;

        MovePoint (45,ang,ang,ang,ang);

        ang = 20;

        MovePoint (90,ang,ang,ang,ang);

        MovePoint (90,ang+90,ang,ang,ang+50);
        MovePoint (90,ang,ang,ang,ang-50);

        MovePoint (90,ang,ang,ang+50,ang);
        MovePoint (90,ang,ang,ang-50,ang);

        ang = 30;

        MovePoint (-90,30,ang,ang+50,ang);

        ang = 40;

        MovePoint (ang,ang,ang,ang-50,ang);
                MovePoint (ang,ang+40,ang,ang+30,ang);
                        MovePoint (ang,ang-40,ang,ang-20,ang);
                                MovePoint (ang+30,ang,ang,ang+50,ang);
                                MovePoint (ang-30,ang,ang,ang,ang);
                                MovePoint (ang,ang,ang,ang+45,ang);
                                MovePoint (ang,ang,ang,ang-45,ang);

        ang = 50;

        MovePoint (ang,ang,ang,ang,ang);

        ang = 80;

        MovePoint (ang,ang,ang,ang-30,ang);

        ang = 20;

        MovePoint (ang,70,ang,ang+20,ang);

        ang = 80;

        MovePoint (50,70,ang,-45,ang);

        ang = 10;

        MovePoint (30,70,ang,45,ang);

    }

public: void MovePoint(float j1, float j2, float j3, float j4, float j5)
    {
        model->GetJointController()->SetPositionTarget(joint[3]->GetScopedName(), j1/180*M_PI);
        model->GetJointController()->SetPositionTarget(joint[4]->GetScopedName(), j2/180*M_PI);
        model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), j3/180*M_PI);
        model->GetJointController()->SetPositionTarget(joint[6]->GetScopedName(), j4/180*M_PI);
        model->GetJointController()->SetPositionTarget(joint[7]->GetScopedName(), j5/180*M_PI);
        sleep(5);
    }

public: void FixAllJoints()
    {
        physics::WorldPtr world = physics::get_world("default");
        physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();

        for(int i=1; i<model->GetJointCount(); ++i)
        {
            //if (i!=6)
            //{
            physics::JointPtr jointToFix;
            jointToFix = engine->CreateJoint("fixed");
            jointToFix->SetName(joint[i]->GetName() + "_fixed_joint");

            // get parent, child, origin can be {0,0,0} in this case
            ignition::math::Pose3d nullPose3d;
            jointToFix->Load(joint[i]->GetParent(), joint[i]->GetChild(), nullPose3d);
            jointToFix->Init();

            fixedJoints.push_back(jointToFix);
            // }
        }
    }

public: void UnfixAllJoints()
    {
        for(int i=1; i<model->GetJointCount(); ++i)
        {
            fixedJoints[i]->Detach();
            fixedJoints[i]->Fini();
        }
    }

public: void UnfixJoint(int number)
    {
        fixedJoints[number-1]->Detach();
        fixedJoints[number-1]->Fini();
    }

public: float findMaxAngleByPeriod(physics::JointPtr &jPtr, int timeSec)
    {
        float max = -9999999;
        ros::Rate loopRate(100);

        for (int j=0; j<100*timeSec; j++)
        {
            ros::spinOnce();
            loopRate.sleep();

            //   cout << jPtr->GetAngle(0).Degree() << endl;

            if (max < jPtr->GetAngle(0).Degree())
            {
                max = jPtr->GetAngle(0).Degree();
            }
        }

        return max;
    }

public: void testPID(physics::JointPtr &jPtr, common::PID testPID, int numberOfSteps = 9)
    {
        cout << endl << "TEST PID " << testPID.GetPGain() << " " << testPID.GetIGain() << " " << testPID.GetDGain() << endl;
        model->GetJointController()->SetPositionPID(jPtr->GetScopedName(), testPID);

        model->GetJointController()->SetPositionTarget(jPtr->GetScopedName(), 0);
        sleep(3);

        for(int i=1; i< numberOfSteps + 1; ++i)
        {
            model->GetJointController()->SetPositionTarget(jPtr->GetScopedName(), 45*i*M_PI/180);
            float overshoot = findMaxAngleByPeriod(jPtr, 5);
            cout << "Overshoot " <<  overshoot - 45*i << endl;
            cout << "Static error " << jPtr->GetAngle(0).Degree() - 45*i << endl << endl;
        }
    }

public: void asyncThread()
    {
        MovePath();
        exit(0);

        ROS_INFO("Thread started");

        int jointNumber = 7;

        UnfixJoint(jointNumber);
        //joint[5]->SetParam("friction", 0, 99999999999);
        joint[3]->SetEffortLimit(0, 80);
        joint[4]->SetEffortLimit(0, 80);
        joint[5]->SetEffortLimit(0, 20);
        joint[6]->SetEffortLimit(0, 20);
        joint[7]->SetEffortLimit(0, 20);
        //joint[jointNumber]->SetEffortLimit(0, 7);
        joint[jointNumber]->SetDamping(0, 1);

        /*    joint[5] = model->GetJoints()[5];
        joint[5]->SetUpperLimit(0, 1000);
        joint[5]->SetLowerLimit(0, -1000);
        joint[5]->SetParam("friction", 0, 0);
        joint[5]->Update();*/

        // Kp = 60 Ki = 190.476 Kd = 4.725
        //        common::PID goodPID(60, 190.476, 4.725);
        /*    common::PID goodPID(720, 2666.67, 48.6);
        //6.6, 17.3684, 0.627)


        model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), goodPID);
        model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), 45*M_PI/180);

        findMaxAngleByPeriod(joint[5], 5);
        cout << "ang " << joint[5]->GetAngle(0).Degree() << endl << endl;

        model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), goodPID);
        model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), 90*M_PI/180);

        findMaxAngleByPeriod(joint[5], 5);
        cout << "ang " << joint[5]->GetAngle(0).Degree() << endl << endl;

        model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), goodPID);
        model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), 135*M_PI/180);

        findMaxAngleByPeriod(joint[5], 5);
        cout << "ang " << joint[5]->GetAngle(0).Degree() << endl << endl;

        model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), goodPID);
        model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), 180*M_PI/180);

        findMaxAngleByPeriod(joint[5], 5);
        cout << "ang " << joint[5]->GetAngle(0).Degree() << endl << endl;

        model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), goodPID);
        model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), 225*M_PI/180);

        findMaxAngleByPeriod(joint[5], 5);
        cout << "ang " << joint[5]->GetAngle(0).Degree() << endl << endl;

        model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), goodPID);
        model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), 270*M_PI/180);

        findMaxAngleByPeriod(joint[5], 5);
        cout << "ang " << joint[5]->GetAngle(0).Degree() << endl << endl;

        exit(0);*/


        /*      ros::Rate loopRate1(50);
        ros::Time rosTimeStart, rosTimeEnd;
        rosTimeStart = ros::Time::now();


        for (int i=0; i<1000; i++)
        {

            ros::spinOnce();
            loopRate1.sleep();
        }

cout << "Time " << ros::Time::now().toSec() - rosTimeStart.toSec() << endl;


      //  sleep(10);

           cout << "expectedCycleTime " << loopRate1.expectedCycleTime().toSec() << endl;



        exit(0);*/

        // joint[5]->SetParam("friction", 0, 0.99);
        //joint[5]->Update();



        vector <common::PID> res;
        common::PID tempPID;
        float Kp_max{210}, Kp_min{150}, Kp_step{20}, Kp_cur{0}, angStart{0}, angFinal{180}, initialError{0}, newError{0}, Tu{0};
        int oscillationCount{0}, numLoops{1000};
        std::vector<double> oscillationTimes(3);
        ros::Rate loopRate(100);
        bool foundRes{false};

        for (Kp_cur = Kp_min; Kp_cur <= Kp_max; Kp_cur += Kp_step)
        {
            oscillationCount = 0;

            //            model->GetJointController()->SetPositionPID(joint[jointNumber]->GetScopedName(), goodPID);
            //            model->GetJointController()->SetPositionTarget(joint[jointNumber]->GetScopedName(), angStart);

            /* joint[5]->SetParam("friction", 999999999, 0);
            sleep(1);
            joint[5]->SetParam("friction", 0, 0);*/



            model->GetJointController()->SetPositionPID(joint[jointNumber]->GetScopedName(), common::PID(Kp_cur, 0, 0));
            model->GetJointController()->SetPositionTarget(joint[jointNumber]->GetScopedName(), angStart);
            sleep(5);
            model->GetJointController()->SetPositionTarget(joint[jointNumber]->GetScopedName(), angFinal*M_PI/180);



            for (int i=0; i<numLoops; i++)
            {


                //  joint[5]->get

                ros::spinOnce();
                loopRate.sleep();

                if (i == 0) // Get the sign of the initial error
                {
                    initialError = angFinal - joint[jointNumber]->GetAngle(0).Degree();
                }

                newError = angFinal - joint[jointNumber]->GetAngle(0).Degree();

                if (signbit(initialError) != signbit(newError) )
                {
                    oscillationTimes.at(oscillationCount) = loopRate.expectedCycleTime().toSec()*i; // Record the time to calculate a period, Tu
                    oscillationCount++;

                    initialError = newError; // Reset to look for another oscillation

                    // If the system is definitely oscillating about the setpoint
                    if ( oscillationCount > 2 )
                    {
                        // Now calculate the period of oscillation (Tu)
                        Tu = oscillationTimes.at(2) - oscillationTimes.at(0);
                        ROS_INFO_STREAM( "Tu (oscillation period): " << Tu );

                        if ( Tu > 3.*loopRate.expectedCycleTime().toSec() )
                        {
                            tempPID.SetPGain(0.6*Kp_cur);
                            tempPID.SetIGain(1.2*Kp_cur/Tu);
                            tempPID.SetDGain(3*Kp_cur*Tu/40.);

                            /*  sleep(5);
                            cout << "error " << abs(joint[5]->GetAngle(0).Degree() - angFinal) << endl;

                            if (abs(joint[5]->GetAngle(0).Degree() - angFinal) < 1 )*/
                            res.push_back(tempPID);

                            foundRes = true;
                            goto DONE;
                        }
                        else
                            break; // Try the next Kp
                    }
                }
            }

DONE:

            if (foundRes)
            {
                //   cout << "Final. Kp = " << tempPID.GetPGain() << " Ki = " << tempPID.GetIGain() << " Kd = " << tempPID.GetDGain() << endl;
                testPID(joint[jointNumber], tempPID, 3);
            }
            else
                cout << "No solution" << endl;





            /*    model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), ang*M_PI/180);
            model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), common::PID(50, 0, 0));

            sleep(1);

            ++ang;

            ROS_INFO("Step");*/
        }

        /*
        //Find best PID
        vector <float> maxVal;

        for (int i=0; i<res.size(); i++)
        {
            model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), goodPID);
            model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), angStart);

            sleep(7);

            model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), res[i]);
            model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), angFinal*M_PI/180);

            ros::Rate loopRate1(100);
            //  ros::Time rosTimeStart, rosTimeEnd;
            //rosTimeStart = ros::Time::now();

            float max = 0;
            for (int j=0; j<1000; j++)
            {
                ros::spinOnce();
                loopRate1.sleep();

                if (max < joint[5]->GetAngle(0).Degree())
                {
                    max = joint[5]->GetAngle(0).Degree();
                }
            }

            cout << "PID " << i << " error " << abs(joint[5]->GetAngle(0).Degree() - 180) << " deg" << endl;

            if (abs(joint[5]->GetAngle(0).Degree() - 180) > 1 )
            {
                max = 0;
                //       ROS_INFO("bad res");
            }

            maxVal.push_back(max);
        }
        */
        /*
        int bestNumb = 0;
        float bestRes = 180;
        for (int i=0; i<maxVal.size(); i++)
        {
            cout << "max val " << maxVal[i] << endl;

            if (bestRes > abs(angFinal - maxVal[i]))
            {
                bestRes = abs(angFinal - maxVal[i]);
                bestNumb = i;
            }
        }

        cout << "bestRes " <<bestRes << " bestNumb " << bestNumb << endl;
        cout << "Final. Kp = " << res[bestNumb].Kp << " Ki = " << res[bestNumb].Ki << " Kd = " << res[bestNumb].Kd << endl;
*/


        ROS_INFO("Thread end");





    }


public: void SetJointPosition(double val)
    {
        //  cout << "Set JOINT " << val << endl;
    /*    ROS_INFO("SetJointPosition");


        model->GetJointController()->SetJointPosition(joint[5]->GetScopedName(), 0*M_PI/180);

        model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), 0);
        model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), common::PID(50, 0, 0));

        cout << joint[5]->Position() << endl;

        double Kp_max = 10.;
        double Kp_min = 0.5;
        double Kp_step = 1.0;

        for (double Kp = Kp_min; Kp <= Kp_max; Kp += Kp_step)
        {

        }*/
        //  cout << joint[5]->GetAngle(0) << endl;

        //  usleep(10000);


        /*LAST STABLE
        for(int i=1;i<model->GetJointCount();++i)
        {
            ++j1;
            joint[i]->Reset();
            joint[i]->Update();
            model->GetJointController()->SetJointPosition(joint[i]->GetScopedName(), j1);
            joint[i]->SetHighStop(0,j1);
            joint[i]->SetLowStop(0,j1);
        }
        */






        // sleep(1);
        /*  initPose.pos.z = 1;
        model->SetWorldPose(initPose);
        //model->SetStatic(true);
        cout << "is static " << model->IsStatic() << endl;
        //model->AttachStaticModel(model, initPose);


        //model->SetGravityMode(false);
        //model->


        /*
        // math::Pose initPose;
        initPose.pos.z = 0.25;
        initPose.pos.y -= 0.01;
        model->SetWorldPose(initPose);
        model->SetStatic(true);*/

        //model->AttachStaticModel(model, initPose);

        /*  for (int i=58; i<model->GetJointCount(); ++i)
        {
            if (i!=1)
                model->GetJointController()->SetJointPosition(joint[i]->GetScopedName(), val*M_PI/180);

        }

        if (j1>=65)
            j1_step = -1;
        if (j1<=-65)
            j1_step = 1;

        j1 += j1_step;

        model->GetJointController()->SetJointPosition(joint[1]->GetScopedName(), j1*M_PI/180);*/

    }
    /*
public: void SetJointPosition(double val)
    {
            cout << "Set Joint " << val << endl;
            model->SetStatic(false);

            model->SetGravityMode(false);


           // math::Pose initPose;
            initPose.pos.z = 0.25;
            initPose.pos.y -= 0.01;
            model->SetWorldPose(initPose);

        for (int i=1; i<model->GetJointCount(); ++i)
        {
            if (i!=1)
            model->GetJointController()->SetJointPosition(joint[i]->GetScopedName(), val*M_PI/180);

        }

        if (j1>=65)
                j1_step = -1;
        if (j1<=-65)
                j1_step = 1;

        j1 += j1_step;

        model->GetJointController()->SetJointPosition(joint[1]->GetScopedName(), j1*M_PI/180);

model->SetStatic(true);
    }*/

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
public: void SetJoint(double _vel)
    {
        //        std::map<std::string, double> positions = this->model->GetJointController()->GetPositions();
        //        //cout << i << " " << this->joint[i]->GetAngle(i) << endl;
        //        dump_map(positions);
        //        cout << endl;

        //for(int i=1;i<model->GetJointCount();++i)
        // for(int i=2;i<3;++i)
        {
            // Set the joint's target velocity.
            //_vel*=M_PI/180;


            model->GetJointController()->SetPositionTarget(joint[4]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[4]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[3]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[3]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[5]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[5]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[6]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[6]->GetScopedName(), common::PID(50, 0, 1));

            model->GetJointController()->SetPositionTarget(joint[7]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[7]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[8]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[8]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[9]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[9]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[10]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[10]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[11]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[11]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[12]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[12]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[13]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[13]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[14]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[14]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[15]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[15]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[16]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[16]->GetScopedName(), common::PID(50, 0, 0));


            model->GetJointController()->SetPositionTarget(joint[25]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[25]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[26]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[26]->GetScopedName(), common::PID(50, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[1]->GetScopedName(), 0);
            model->GetJointController()->SetPositionPID(joint[1]->GetScopedName(), common::PID(1000, 0, 0));

            model->GetJointController()->SetPositionTarget(joint[2]->GetScopedName(), _vel*M_PI/180);
            model->GetJointController()->SetPositionPID(joint[2]->GetScopedName(), common::PID(10000, 100, 10));
            //model->GetJointController()->SetPositionPID(joint[i]->GetScopedName(), common::PID(1000, 100, 10));

            //if(!this->model->GetJointController()->SetPositionTarget(this->joint[i]->GetScopedName(), _vel))
            // std::cout << "FUCKING ERROR\n";
        }

        math::Angle ang = joint[2]->GetAngle(0);
        std::cout << "ERROR = " << _vel - ang.Degree() << " FROM URDF\n";
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
private: void OnMsg(ConstVector3dPtr &_msg)
    {
        this->SetJointPosition(_msg->x());
        //this->SetJoint(_msg->x());
    }
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}

















#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <math.h>

// Use dynamic_reconfigure to adjust Kp, Ki, and Kd
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

void setKiKdToZero();
void setKp(double Kp);
void setpoint_callback(const std_msgs::Float64& setpoint_msg);
void state_callback(const std_msgs::Float64& state_msg);
void setFinalParams();

namespace autotune
{
double Ku = 0.;
double Tu = 0.;
double setpoint = 0.;
double state = 0.;
std::string nameSpc = "/left_wheel_pid/";
int oscillationCount = 0;
int numLoops = 100; // Will look for oscillations for numLoops*loopRate
int initialError = 0;
double Kp_ZN = 0.;
double Ki_ZN = 0.;
double Kd_ZN = 0.;
bool foundKu = false;
std::vector<double> oscillationTimes(3); // Used to calculate Tu, the oscillation period
}

int tune(int argc, char** argv) {

    ros::init(argc, argv, "autotune_node");
    ros::NodeHandle autotuneNode;
    ros::start();
    ros::Subscriber setpoint_sub = autotuneNode.subscribe("/setpoint", 1, setpoint_callback );
    ros::Subscriber state_sub = autotuneNode.subscribe("/state", 1, state_callback );
    ros::Rate loopRate(50);

    // Set Ki and Kd to zero for the ZN method with dynamic_reconfigure
    setKiKdToZero();

    // Define how rapidly the value of Kp is varied, and the max/min values to try
    double Kp_max = 10.;
    double Kp_min = 0.5;
    double Kp_step = 1.0;

    for (double Kp = Kp_min; Kp <= Kp_max; Kp += Kp_step)
    {
        //////////////////////
        // Get the error sign.
        //////////////////////
        // Need to wait for new setpoint/state msgs
        ros::topic::waitForMessage<std_msgs::Float64>("setpoint");
        ros::topic::waitForMessage<std_msgs::Float64>("state");

        // Try a new Kp.
        setKp(Kp);
        ROS_INFO_STREAM("Trying Kp = " << Kp); // Blank line on terminal
        autotune::oscillationCount = 0; // Reset to look for oscillations again

        for (int i=0; i<autotune::numLoops; i++) // Collect data for loopRate*numLoops seconds
        {
            ros::spinOnce();
            loopRate.sleep();
            if (i == 0) // Get the sign of the initial error
            {
                autotune::initialError = (autotune::setpoint - autotune::state);
            }

            // Did the system oscillate about the setpoint? If so, Kp~Ku.
            // Oscillation => the sign of the error changes
            // The first oscillation is caused by the change in setpoint. Ignore it. Look for 2 oscillations.
            // Get a fresh state message
            ros::topic::waitForMessage<std_msgs::Float64>("state");
            double newError = (autotune::setpoint - autotune::state); //Sign of the error
            //ROS_INFO_STREAM("New error: "<< newError);
            if ( std::signbit(autotune::initialError) != std::signbit(newError) )
            {
                autotune::oscillationTimes.at(autotune::oscillationCount) = loopRate.expectedCycleTime().toSec()*i; // Record the time to calculate a period, Tu
                autotune::oscillationCount++;
                ROS_INFO_STREAM("Oscillation occurred. Oscillation count:  " << autotune::oscillationCount);
                autotune::initialError = newError; // Reset to look for another oscillation

                // If the system is definitely oscillating about the setpoint
                if ( autotune::oscillationCount > 2 )
                {
                    // Now calculate the period of oscillation (Tu)
                    autotune::Tu = autotune::oscillationTimes.at(2) - autotune::oscillationTimes.at(0);
                    ROS_INFO_STREAM( "Tu (oscillation period): " << autotune::Tu );
                    //ROS_INFO_STREAM( "2*sampling period: " << 2.*loopRate.expectedCycleTime().toSec() );

                    // We're looking for more than just the briefest dip across the setpoint and back.
                    // Want to see significant oscillation
                    if ( autotune::Tu > 3.*loopRate.expectedCycleTime().toSec() )
                    {
                        autotune::Ku = Kp;

                        // Now calculate the other parameters with ZN method
                        autotune::Kp_ZN = 0.6*autotune::Ku;
                        autotune::Ki_ZN = 1.2*autotune::Ku/autotune::Tu;
                        autotune::Kd_ZN = 3.*autotune::Ku*autotune::Tu/40.;

                        autotune::foundKu = true;
                        goto DONE;
                    }
                    else
                        break; // Try the next Kp
                }
            }
        }
    }
DONE:

    if (autotune::foundKu == true)
    {
        setFinalParams();
    }
    else
        ROS_INFO_STREAM("Did not see any oscillations for this range of Kp. Adjust Kp_max and Kp_min to broaden the search.");

    ros::shutdown();
    return 0;
}

// Set Ki and Kd to zero with dynamic_reconfigure
void setKiKdToZero()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;
    double_param.name = "Ki";
    double_param.value = 0.0;
    config.doubles.push_back(double_param);
    double_param.name = "Kd";
    double_param.value = 0.0;
    config.doubles.push_back(double_param);
    srv_req.config = config;
    ros::service::call(autotune::nameSpc + "set_parameters", srv_req, srv_resp);
}

// Set Kp with dynamic_reconfigure
void setKp(double Kp)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;

    // A blank service call to get the current parameters into srv_resp
    ros::service::call(autotune::nameSpc + "set_parameters", srv_req, srv_resp);

    double_param.name = "Kp";
    double_param.value = Kp / srv_resp.config.doubles.at(0).value; // Adjust for the scale slider on the GUI
    config.doubles.push_back(double_param);
    srv_req.config = config;
    ros::service::call(autotune::nameSpc + "set_parameters", srv_req, srv_resp);
}

void setpoint_callback(const std_msgs::Float64& setpoint_msg)
{
    autotune::setpoint = setpoint_msg.data;
}

void state_callback(const std_msgs::Float64& state_msg)
{
    autotune::state = state_msg.data;
    //ROS_INFO_STREAM(autotune::state);
}

// Print out and set the final parameters as calculated by the autotuner
void setFinalParams()
{
    ros::NodeHandle nh1("gazebo_ros_control");
    ros::Timer timer1 = nh1.createTimer(ros::Duration(0.1), timerCallback);

    ROS_INFO_STREAM(" ");
    ROS_INFO_STREAM("The suggested parameters are: ");
    ROS_INFO_STREAM("Kp  "<< autotune::Kp_ZN);
    ROS_INFO_STREAM("Ki  "<< autotune::Ki_ZN);
    ROS_INFO_STREAM("Kd  "<< autotune::Kd_ZN);

    // Set the ZN parameters with dynamic_reconfigure
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config config;

    // A blank service call to get the current parameters into srv_resp
    ros::service::call(autotune::nameSpc + "set_parameters", srv_req, srv_resp);

    double_param.name = "Kp";
    double_param.value = autotune::Kp_ZN / srv_resp.config.doubles.at(0).value; // Adjust for the scale slider on the GUI
    config.doubles.push_back(double_param);

    double_param.name = "Ki";
    double_param.value = autotune::Ki_ZN / srv_resp.config.doubles.at(0).value; // Adjust for the scale slider on the GUI
    config.doubles.push_back(double_param);

    double_param.name = "Kd";
    double_param.value = autotune::Kd_ZN / srv_resp.config.doubles.at(0).value; // Adjust for the scale slider on the GUI
    config.doubles.push_back(double_param);

    srv_req.config = config;
    ros::service::call(autotune::nameSpc + "set_parameters", srv_req, srv_resp);
}






#endif






