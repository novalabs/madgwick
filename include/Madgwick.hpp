/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/utils/BasicSensor.hpp>
//#include <core/mw/Callback.hpp>

#include <ModuleConfiguration.hpp>

#include <core/sensor_msgs/RPY_f32.hpp>
#include <core/madgwick/MadgwickConfiguration.hpp>

namespace core {
namespace madgwick {
class Madgwick:
   public core::mw::CoreNode,
   public core::mw::CoreConfigurable<core::madgwick::MadgwickConfiguration>
{
public:
   Madgwick(
      const char*                name,
      core::os::Thread::Priority priority = core::os::Thread::PriorityEnum::NORMAL
   );
   virtual
   ~Madgwick();

private:
   ModuleConfiguration::L3GD20H_GYRO_DATATYPE _gyroData;
   ModuleConfiguration::LSM303D_ACC_DATATYPE  _accData;
   ModuleConfiguration::LSM303D_MAG_DATATYPE  _magData;

private:
   bool
   onPrepareMW();

   bool
   onLoop();

   static bool
   gyroCallback(
      const ModuleConfiguration::L3GD20H_GYRO_DATATYPE& msg,
      void*                                             context
   );

   static bool
   accCallback(
      const ModuleConfiguration::LSM303D_ACC_DATATYPE& msg,
      void*                                            context
   );

   static bool
   magCallback(
      const ModuleConfiguration::LSM303D_MAG_DATATYPE& msg,
      void*                                            context
   );


private:
   core::mw::Subscriber<ModuleConfiguration::L3GD20H_GYRO_DATATYPE, 2> _subscriberGyro;
   core::mw::Subscriber<ModuleConfiguration::LSM303D_ACC_DATATYPE, 2>  _subscriberAcc;
   core::mw::Subscriber<ModuleConfiguration::LSM303D_MAG_DATATYPE, 2>  _subscriberMag;
   core::mw::Publisher<sensor_msgs::RPY_f32> _publisher;
};
}
}
