#include "smb_battery_payload/smb_batt_pay_display.hpp"
#include <pluginlib/class_list_macros.h>
#include <QHBoxLayout>

namespace smb_rviz_plugins {
using namespace rviz;

SMBBatteryDisplay::SMBBatteryDisplay(){

    visual_category_  = new rviz::Property("Visualize", QVariant(), "", this);

    battery_topic_ = new rviz::RosTopicProperty("Topic Battery", "",
                                            QString::fromStdString(ros::message_traits::datatype<sensor_msgs::BatteryState>()),
                                            "sensor_msgs::BatteryState topic to subscribe to.",
                                            this, SLOT(updateTopic()));

    visual_battery_1_ = new rviz::BoolProperty("Show Battery 1", true, "Set visibility of Battery 1",
                                            visual_category_, SLOT(changeVisibility()),this);

    visual_battery_2_ = new rviz::BoolProperty("Show Battery 2", true, "Set visibility of Battery 2",
                                            visual_category_, SLOT(changeVisibility()),this);

    visual_battery_3_ = new rviz::BoolProperty("Show Battery 3", true, "Set visibility of Battery 3",
                                            visual_category_, SLOT(changeVisibility()),this);                                    

}

void SMBBatteryDisplay::setTopic(const QString& topic, const QString& datatype){
    battery_topic_->setString(topic);
}

void SMBBatteryDisplay::onInitialize(){
    // Create widget to add to rviz
    QWidget * parent = new QWidget;
    auto layout = new QHBoxLayout;
    
    // Add 3 battery widgets for the 3 batteries
    battery_1_panel_ = new BatteryPanel("Battery 1");
    battery_2_panel_ = new BatteryPanel("Battery 2");
    battery_3_panel_ = new BatteryPanel("Battery 3");
    
    // Add to the layout
    layout->addWidget(battery_1_panel_);
    layout->addWidget(battery_2_panel_);
    layout->addWidget(battery_3_panel_);
    parent->setLayout(layout);
    
    setAssociatedWidget(parent);

}

void SMBBatteryDisplay::onEnable(){
    subscribe();
}

void SMBBatteryDisplay::onDisable(){
    unsubscribe();
}

void SMBBatteryDisplay::changeVisibility(){

    battery_1_panel_->setVisible(visual_battery_1_->getBool());
    battery_2_panel_->setVisible(visual_battery_2_->getBool());
    battery_3_panel_->setVisible(visual_battery_3_->getBool());

}

void SMBBatteryDisplay::updateTopic(){
    unsubscribe();
    reset();
    subscribe();
}

void SMBBatteryDisplay::subscribe(){
    if(!isEnabled())
        return;
    
    std::string topic_name = battery_topic_->getTopicStd();
    if (topic_name.empty())
    {
        setStatus(StatusProperty::Error, "Output Topic", "No topic set");
        return;
    }

    std::string error;
    if (!ros::names::validate(topic_name, error))
    {
        setStatus(StatusProperty::Error, "Output Topic", QString(error.c_str()));
        return;
    }

    try
    {
        battery_subscriber_ = update_nh_.subscribe(topic_name, 1, &SMBBatteryDisplay::batteryMsgCallback, this);
        setStatus(StatusProperty::Ok, "Battery", "OK");
    }
    catch (ros::Exception& e)
    {
        setStatus(StatusProperty::Error, "Battery", QString("Error subscribing: ") + e.what());
    }
}

void SMBBatteryDisplay::unsubscribe(){
    battery_subscriber_.shutdown();
}

void SMBBatteryDisplay::batteryMsgCallback(const sensor_msgs::BatteryStateConstPtr &msg){
    BatteryPanel * battery_panel = nullptr;
    if(msg->location == "Battery 1")
    {
        battery_panel = battery_1_panel_;
    }else if (msg->location == "Battery 2")
    {
        battery_panel = battery_2_panel_;
    }else if (msg->location == "ACDC Connector")
    {
        battery_panel = battery_3_panel_;
    }else{
        return;
    }
    
    battery_panel->setPercentage(msg->percentage);
    battery_panel->setVoltage(msg->voltage);
    if(!msg->present){
        battery_panel->setBatteryStatus(BatteryPanel::BatteryStatus::Missing);
    }else{
        switch (msg->power_supply_status)
        {
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
                battery_panel->setBatteryStatus(
                    BatteryPanel::BatteryStatus::Charging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL:
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
                battery_panel->setBatteryStatus(
                    BatteryPanel::BatteryStatus::Discharging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN:
                battery_panel->setBatteryStatus(
                    BatteryPanel::BatteryStatus::Unknown);
                break;
        }
    }

}


}

PLUGINLIB_EXPORT_CLASS(smb_rviz_plugins::SMBBatteryDisplay, rviz::Display)