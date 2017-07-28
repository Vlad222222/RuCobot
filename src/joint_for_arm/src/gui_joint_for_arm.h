#ifndef GUI_JOINT_FOR_ARM
#define GUI_JOINT_FOR_ARM

#include <QObject>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QDebug>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "boost/thread.hpp"

class  gui_joint_for_arm : public QWidget
{
    Q_OBJECT
public:
     gui_joint_for_arm();
     ~gui_joint_for_arm();

private:
     ros::Publisher jointStatePub;
     QVector<QString> m_baseNameJoint;
     QVector<QSlider*> m_sliders;
     QVector<QDoubleSpinBox*> m_spinBoxes;

     void createGuiForLeftArm();
     void createGuiForRightArm();
     void publishDate(double value, std::string name);
     void rosThread();

private slots:
     void changeValueSpinBox();
     void changeValueSlider();
};

#endif
