#include <ros/ros.h>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/sensors.hh>

#include <uwb_gazebo_plugin/uwbPose.h>
#include <mrs_msgs/RangeWithCovarianceArrayStamped.h>

#include "uwb_range.h"

namespace gazebo
{
    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    UwbRange::UwbRange()
    {
        return;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    UwbRange::~UwbRange()
    {
        return;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the controller
    void UwbRange::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        this->parentSensor =
            std::dynamic_pointer_cast<sensors::WirelessTransmitter>(_parent);

        // Make sure the parent sensor is valid.
        if (!this->parentSensor)
        {
            gzerr << "WifiReceiverPlugin requires a Wireless Transmitter Sensor.\n";
            return;
        }

        // // Connect to the sensor update event.
        // this->updateConnection = this->parentSensor->ConnectUpdated(
        //     std::bind(&WifiReceiverPlugin::UpdateImpl, this));

        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);

        std::cout << "Initialize UWB range Gazebo plugin" << std::endl;
        std::cout << this->parentSensor->Freq() << std::endl;

        this->world = physics::get_world("default");

        this->nh_ = ros::NodeHandle("~");

        std::string result_topic = "";

        if(_sdf->HasElement("publish_topic"))
            result_topic = _sdf->GetElement("publish_topic")->Get<std::string>();

        this->uwb_pub = this->nh_.advertise<uwb_gazebo_plugin::uwbPose>("uwb_notify", 1);
        this->uwb_sub = this->nh_.subscribe("uwb_notify", 10, &UwbRange::uwbCallback, this);

        this->result_pub = this->nh_.advertise<mrs_msgs::RangeWithCovarianceArrayStamped>(result_topic, 1);

        this->uwb_id = 0;

        if (_sdf->HasElement("uwb_id"))
            this->uwb_id = _sdf->GetElement("uwb_id")->Get<int>();

        if(_sdf->HasElement("frame_name"))
            this->uwb_frame = _sdf->GetElement("frame_name")->Get<std::string>();

        std::cout << "UWB ID: " << uwb_id << std::endl;

        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(
            std::bind(&UwbRange::Update, this));

        // std::cout << this->
        // Make sure the ROS node for Gazebo has already been initalized
        return;
    }

    void UwbRange::uwbCallback(const uwb_gazebo_plugin::uwbPose::ConstPtr &msg)
    {
        if (msg->uwb_id == this->uwb_id)
            return;

        physics::EntityPtr entity = this->world->EntityByName(this->parentSensor->ParentName());
        auto pose = (entity.get()->WorldPose() + this->parentSensor->Pose()).Pos();

        std::cout << "UWB callback" << std::endl;

        double dBm = this->parentSensor->SignalStrength(ignition::math::Pose3d(msg->x.data, msg->y.data, msg->z.data, 0., 0., 0.), this->parentSensor->Gain());
        double distance = (pose - ignition::math::Vector3d(msg->x.data, msg->y.data, msg->z.data)).Length();

        mrs_msgs::RangeWithCovarianceArrayStamped result;

        result.header.stamp = ros::Time::now();
        result.header.frame_id = "uwb_" + std::to_string(this->uwb_id);
        result.header.frame_id = this->uwb_frame;

        mrs_msgs::RangeWithCovarianceIdentified range;

        range.id = msg->uwb_id;
        range.power_a = dBm;
        range.power_b = dBm;

        range.range.field_of_view = 2*M_PI;
        range.range.max_range = 100;
        range.range.min_range = 0;
        range.range.radiation_type = 3;
        range.range.range = distance;
        range.range.header.frame_id = this->uwb_frame;
        range.variance = 0.1;

        result.ranges.push_back(range);

        this->result_pub.publish(result);

        return;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Update the controller
    void UwbRange::Update()
    {
        physics::EntityPtr entity = this->world->EntityByName(this->parentSensor->ParentName());
        auto pose = (entity.get()->WorldPose() + this->parentSensor->Pose()).Pos();

        uwb_gazebo_plugin::uwbPose uwb_pose;

        uwb_pose.x.data = pose[0];
        uwb_pose.y.data = pose[1];
        uwb_pose.z.data = pose[2];

        uwb_pose.uwb_id = this->uwb_id;

        this->uwb_pub.publish(uwb_pose);
        return;
    }
    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(UwbRange)
}