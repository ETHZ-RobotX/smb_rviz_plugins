#ifndef SMB_BATTERY_DISPLAY_HPP__
#define SMB_BATTERY_DISPLAY_HPP__

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <smb_rviz_plugins/SMBPower.h>

#include <smb_battery_payload/smb_batt_panel.hpp>
#include "smb_battery_payload/smb_plug_pay_panel.hpp"

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

    private Q_SLOTS:
        void updateTopic();
        void changeVisibility();
        

    private:
        void subscribe();
        void unsubscribe();
        void batteryMsgCallback(const smb_rviz_plugins::SMBPowerConstPtr &msg);

        rviz::RosTopicProperty* battery_topic_;

        rviz::BoolProperty* visual_battery_1_;
        rviz::BoolProperty* visual_battery_2_;
        rviz::BoolProperty* visual_plug_;

        rviz::Property* visual_category_;
        
        ros::Subscriber battery_subscriber_;
        
        BatteryPanel* battery_1_panel_{nullptr};
        BatteryPanel* battery_2_panel_{nullptr};
        PlugPanel* plug_panel_{nullptr};
};


}

#endif //SMB_BATTERY_DISPLAY_HPP__
