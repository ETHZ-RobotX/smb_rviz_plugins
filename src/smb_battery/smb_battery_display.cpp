#include "smb_battery/smb_battery_display.hpp"
#include <pluginlib/class_list_macros.h>

namespace smb_rviz_plugins {
using namespace rviz;

SMBBatteryDisplay::SMBBatteryDisplay(){

    // TODO: add sensor type
    topic_property_ = new rviz::RosTopicProperty("Topic Battery1", "",
                                            QString::fromStdString(ros::message_traits::datatype<sensor_msgs::BatteryState>()),
                                            "sensor_msgs::BatteryState topic to subscribe to.",
                                            this, SLOT(updateTopic()));
    topic_property_2 = new rviz::RosTopicProperty("Topic Battery2", "",
                                            QString::fromStdString(ros::message_traits::datatype<sensor_msgs::BatteryState>()),
                                            "sensor_msgs::BatteryState topic to subscribe to.",
                                            this, SLOT(updateTopic()));
}

void SMBBatteryDisplay::setTopic(const QString& topic, const QString& datatype){
    topic_property_->setString(topic);
}

void SMBBatteryDisplay::onInitialize(){
    battery_panel_ = new BatteryPanel();
    setAssociatedWidget(battery_panel_);
}

void SMBBatteryDisplay::onEnable(){
    subscribe();
}

void SMBBatteryDisplay::onDisable(){
    unsubscribe();
}

void SMBBatteryDisplay::updateTopic(){
    unsubscribe();
    reset();
    subscribe();
}

void SMBBatteryDisplay::subscribe(){
    if(!isEnabled())
        return;
    
    std::string topic_name = topic_property_->getTopicStd();
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
    battery_panel_->setPercentage(msg->percentage);
    battery_panel_->setVoltage(msg->voltage);
    if(!msg->present){
        battery_panel_->setBatteryStatus(BatteryPanel::BatteryStatus::Missing);
    }else{
        switch (msg->power_supply_status)
        {
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
                battery_panel_->setBatteryStatus(
                    BatteryPanel::BatteryStatus::Charging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL:
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
                battery_panel_->setBatteryStatus(
                    BatteryPanel::BatteryStatus::Discharging);
                break;
            case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN:
                battery_panel_->setBatteryStatus(
                    BatteryPanel::BatteryStatus::Unknown);
                break;
        }
    }

}


}

PLUGINLIB_EXPORT_CLASS(smb_rviz_plugins::SMBBatteryDisplay, rviz::Display)