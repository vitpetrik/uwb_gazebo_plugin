#ifndef GAZEBO_ROS_TEMPLATE_HH
#define GAZEBO_ROS_TEMPLATE_HH

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/WirelessTransmitter.hh>

#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace gazebo
{

   class UwbRange : public SensorPlugin
   {
      /// \brief Constructor
   public:
      UwbRange();

      /// \brief Destructor
   public:
      virtual ~UwbRange();

      /// \brief Load the controller
   public:
      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

   private: sensors::WirelessTransmitterPtr parentSensor;

   private:

      void uwbCallback(const uwb_gazebo_plugin::uwbPose::ConstPtr& msg);

      /// \brief Update the controller
   protected:
      virtual void Update();

   private:
      uint32_t uwb_id;

      std::string uwb_frame;

      ros::NodeHandle nh_;

      ros::Publisher uwb_pub;
      ros::Subscriber uwb_sub;

      ros::Publisher result_pub;

      gazebo::physics::WorldPtr world;

   /// \brief Connection that maintains a link between the contact sensor's
   /// updated signal and the OnUpdate callback.
      private: event::ConnectionPtr updateConnection;

   };

}

#endif
