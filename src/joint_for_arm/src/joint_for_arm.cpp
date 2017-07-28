#include <ros/ros.h>
#include <QApplication>

#include "gui_joint_for_arm.h"

int main(int argc, char** argv)
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "joint_for_arm");

    gui_joint_for_arm gui;
    gui.show();

    return a.exec();
}
