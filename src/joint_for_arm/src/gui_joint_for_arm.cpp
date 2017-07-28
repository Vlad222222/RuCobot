#include "gui_joint_for_arm.h"

gui_joint_for_arm::gui_joint_for_arm() :
    QWidget(0)
{
    ros::NodeHandle node("~");

    int type = node.param("type", 0);
    std::string topic = node.param<std::string>("nametopic", "/joint_states");

    jointStatePub = node.advertise<sensor_msgs::JointState>(topic, 100);

    m_baseNameJoint.push_back("j_bolshoj_osnova");
    m_baseNameJoint.push_back("j_proximal_falang_bolshoj");
    m_baseNameJoint.push_back("j_medial_falanga_bolshoj");
    m_baseNameJoint.push_back("j_distal_falang_bolshoy");

    m_baseNameJoint.push_back("j_sharnir_palec_ukaz");
    m_baseNameJoint.push_back("j_proximal_falang_ukaz");
    m_baseNameJoint.push_back("j_medial_falanga_ukaz");
    m_baseNameJoint.push_back("j_distal_falanga_ukaz");

    m_baseNameJoint.push_back("j_sharnir_palec_sred");
    m_baseNameJoint.push_back("j_proximal_falang_sred");
    m_baseNameJoint.push_back("j_medial_falanga_sred");
    m_baseNameJoint.push_back("j_distal_falanga_sred");

    m_baseNameJoint.push_back("j_sharnir_palec_bezim");
    m_baseNameJoint.push_back("j_proximal_falang_bezim");
    m_baseNameJoint.push_back("j_medial_falanga_bezim");
    m_baseNameJoint.push_back("j_distal_falanga_bezim");

    m_baseNameJoint.push_back("j_sharnir_palec_miz");
    m_baseNameJoint.push_back("j_proximal_falang_miz");
    m_baseNameJoint.push_back("j_medial_falanga_miz");
    m_baseNameJoint.push_back("j_distal_falanga_miz");

    switch (type) {
    case 0:
        createGuiForLeftArm();
        break;
    case 1:
        createGuiForRightArm();
        break;
    default:
        break;
    }

    boost::thread thr(boost::bind(&gui_joint_for_arm::rosThread, this));
    thr.detach();
}

gui_joint_for_arm::~gui_joint_for_arm()
{
    ros::shutdown();
}

void gui_joint_for_arm::rosThread()
{
    while (ros::ok()) {
        ros::spinOnce();
    }
}

void gui_joint_for_arm::createGuiForLeftArm()
{
    QVBoxLayout *mainLayot = new QVBoxLayout();
    mainLayot->setContentsMargins(10,1,10,1);

    for(int i = 0; i < m_baseNameJoint.size(); ++i){
        QString nameJoint = m_baseNameJoint[i] + "_l";
        QLabel *label = new QLabel(nameJoint);
        QSlider *slider = new QSlider(Qt::Horizontal);
        QDoubleSpinBox *spinbox = new QDoubleSpinBox();

        slider->setProperty("name", QVariant(nameJoint));
        slider->setMinimum(-314);
        slider->setMaximum(314);
        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(changeValueSlider()));
        m_sliders.push_back(slider);

        spinbox->setProperty("name", QVariant(nameJoint));
        spinbox->setSingleStep(0.01);
        spinbox->setMinimum(-3.14);
        spinbox->setMaximum(3.14);
        connect(spinbox, SIGNAL(valueChanged(double)), this, SLOT(changeValueSpinBox()));
        m_spinBoxes.push_back(spinbox);

        QHBoxLayout *object = new QHBoxLayout();
        object->setContentsMargins(10,1,10,1);
        object->addWidget(label);;
        object->addWidget(spinbox);
        object->addWidget(slider);

        mainLayot->addLayout(object);
    }

    this->setLayout(mainLayot);
}

void gui_joint_for_arm::createGuiForRightArm()
{
    QVBoxLayout *mainLayot = new QVBoxLayout();
    mainLayot->setContentsMargins(10,1,10,1);

    for(int i = 0; i < m_baseNameJoint.size(); ++i){
        QString nameJoint = m_baseNameJoint[i] + "_r";
        QLabel *label = new QLabel(nameJoint);
        QSlider *slider = new QSlider(Qt::Horizontal, 0);
        QDoubleSpinBox *spinbox = new QDoubleSpinBox();

        slider->setProperty("name", QVariant(nameJoint));
        slider->setMinimum(-314);
        slider->setMaximum(314);
        connect(slider, SIGNAL(valueChanged(int)), this, SLOT(changeValueSlider()));
        m_sliders.push_back(slider);

        spinbox->setProperty("name", QVariant(nameJoint));
        spinbox->setSingleStep(0.01);
        spinbox->setMinimum(-3.14);
        spinbox->setMaximum(3.14);
        connect(spinbox, SIGNAL(valueChanged(double)), this, SLOT(changeValueSpinBox()));
        m_spinBoxes.push_back(spinbox);

        QHBoxLayout *object = new QHBoxLayout();
        object->setContentsMargins(10,1,10,1);
        object->addWidget(label);;
        object->addWidget(spinbox);
        object->addWidget(slider);

        mainLayot->addLayout(object);
    }

    this->setLayout(mainLayot);
}

void gui_joint_for_arm::publishDate(double value, std::__cxx11::string name)
{
    sensor_msgs::JointState state;
    state.header.frame_id = "/base_link";

    state.name.push_back(name);
    state.position.push_back(value);

    jointStatePub.publish(state);
}

void gui_joint_for_arm::changeValueSlider()
{
    QSlider *slider = qobject_cast<QSlider*>(sender());

    QString name = slider->property("name").toString();

    for(int i = 0; i < m_spinBoxes.size(); ++i )
        if(m_spinBoxes[i]->property("name").toString() == name)
            m_spinBoxes[i]->setValue((double)slider->value()/100);

    publishDate((double)slider->value()/100, name.toStdString());
}

void gui_joint_for_arm::changeValueSpinBox()
{
    QDoubleSpinBox *spinBox = qobject_cast<QDoubleSpinBox*>(sender());

    QString name = spinBox->property("name").toString();

    for(int i = 0; i < m_sliders.size(); ++i )
        if(m_sliders[i]->property("name").toString() == name)
            m_sliders[i]->setValue((int)(spinBox->value()*100));

    publishDate(spinBox->value(), name.toStdString());
}
