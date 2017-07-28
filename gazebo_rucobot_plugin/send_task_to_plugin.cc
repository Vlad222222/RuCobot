#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

using namespace std;

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    cout << "Load gazebo as a client" << endl;
    // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
#else
    gazebo::client::setup(_argc, _argv);
#endif

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    while(1)
    {
        // Publish to the velodyne topic
        gazebo::transport::PublisherPtr pub =
                node->Advertise<gazebo::msgs::Vector3d>("~/rucobot/joint");

        // Wait for a subscriber to connect to this publisher
        cout << "Wait for a subscriber to connect to this publisher" << endl;
        pub->WaitForConnection();

        // Create a a vector3 message
        gazebo::msgs::Vector3d msg;

        // Set the velocity in the x-component
#if GAZEBO_MAJOR_VERSION < 6
        gazebo::msgs::Set(&msg, gazebo::math::Vector3(std::atof(_argv[1]), 0, 0));
#else
        gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));
#endif

        cout << "Send the message" << endl;
        // Send the message
        pub->Publish(msg);
        usleep(100000);
    }

    // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
#else
    gazebo::client::shutdown();
#endif
}
