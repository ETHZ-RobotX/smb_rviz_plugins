#ifndef SMB_BATTERY_DISPLAY_HPP__
#define SMB_BATTERY_DISPLAY_HPP__

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <sensor_msgs/BatteryState.h>

#include <smb_battery/smb_battery_panel.hpp>

namespace smb_rviz_plugins {

class SMBBatteryDisplay : public rviz::Display {
    Q_OBJECT
    public:

        SMBBatteryDisplay();

        void setTopic( const QString& topic, const QString& datatype ) override;
    
    protected:

        void onInitialize() override;
        void onEnable() override;
        void onDisable() override;
    
    protected Q_SLOTS:
        void updateTopic();

    private:
        void subscribe();
        void unsubscribe();
        void batteryMsgCallback(const sensor_msgs::BatteryStateConstPtr &msg);

        rviz::RosTopicProperty* topic_property_;
        rviz::RosTopicProperty* topic_property_2;
        ros::Subscriber battery_subscriber_;
        
        BatteryPanel* battery_panel_{nullptr};
};


}

#endif //SMB_BATTERY_DISPLAY_HPP__
