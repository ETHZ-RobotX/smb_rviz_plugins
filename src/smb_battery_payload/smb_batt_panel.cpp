#include <smb_battery_payload/smb_batt_panel.hpp>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QPen>
#include <iostream>

namespace smb_rviz_plugins{

BatteryPanel::BatteryPanel(QString name, QWidget *parent)
    :QWidget(parent){

    QVBoxLayout* layout = new QVBoxLayout;
    battery_name_= new QLabel(name);
    battery_name_->setStyleSheet("font-weight: bold");
    layout->addWidget(battery_name_);

    QHBoxLayout* layout_indicator = new QHBoxLayout;

    battery_icon_ = new QLabel();
    setIcon(":/battery/battery_warning.svg");
    layout_indicator->addWidget(battery_icon_);

    battery_text_ = new QLabel("No Data");
    layout_indicator->addWidget(battery_text_);

    layout_indicator->addStretch();

    layout->addLayout(layout_indicator);

    setLayout(layout);
}

void BatteryPanel::setBatteryStatus(BatteryPanel::BatteryStatus status){
    battery_status_ = status;
    updateWidgets();
}

void BatteryPanel::setPercentage(double percentage){
    percentage_ = percentage;
    updateWidgets();
}

void BatteryPanel::setVoltage(double voltage){
    voltage_ = voltage;
    updateWidgets();
}

void BatteryPanel::setIcon(const QString &path){
    QPixmap pixmap(path);
    const auto& font_metrics = battery_icon_->fontMetrics();
    auto icon_width = font_metrics.averageCharWidth() * 6;
    auto icon_height = font_metrics.height()*2;
    battery_icon_->setPixmap(pixmap.scaled(icon_width, icon_height, Qt::KeepAspectRatio));
}

void BatteryPanel::setInUse(bool in_use){
    auto pixmap = battery_icon_->pixmap()->toImage();
    QPainter painter(&pixmap);
    QPen pen;
    if(in_use){
        pen.setColor(Qt::green);
    }else{
        pen.setColor(Qt::red);
    }
    pen.setWidth(15);
    painter.setPen(pen);
    painter.drawPoint(5,5);
    painter.end();
    battery_icon_->setPixmap(QPixmap::fromImage(pixmap));
}

void BatteryPanel::updateWidgets(){
    double percentage = percentage_ * 100;
    bool battery_in_use = false;
    battery_text_->setText(QString("%1% (%2 V)").arg(percentage).arg(voltage_));
    switch(battery_status_)
    {
        case BatteryStatus::Unknown:
            setIcon(":/battery/battery_warning.svg");
            setInUse(battery_in_use);
            break;
        case BatteryStatus::Charging:
            setIcon(":/battery/battery_charge.svg");
            setInUse(battery_in_use);
            break;
        case BatteryStatus::Discharging:
            battery_in_use = true;
        case BatteryStatus::NotCharging:
            if(percentage_ < 0.1)
                setIcon(":/battery/battery_0.svg");
            else if(percentage_ < 0.2)
                setIcon(":/battery/battery_1.svg");
            else if(percentage_ < 0.3)
                setIcon(":/battery/battery_2.svg");
            else if(percentage_ < 0.4)
                setIcon(":/battery/battery_3.svg");
            else if(percentage_ < 0.5)
                setIcon(":/battery/battery_4.svg");
            else if(percentage_ < 0.6)
                setIcon(":/battery/battery_5.svg");
            else if(percentage_ < 0.7)
                setIcon(":/battery/battery_6.svg");
            else if(percentage_ < 0.8)
                setIcon(":/battery/battery_7.svg");
            else if(percentage_ < 0.9)
                setIcon(":/battery/battery_8.svg");
            else
                setIcon(":/battery/battery_full.svg");
            setInUse(battery_in_use);
            break;
        case BatteryStatus::Missing:
            setIcon(":/battery/battery_warning.svg");
            setInUse(battery_in_use);
            break;
    }
}


}