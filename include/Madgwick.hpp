/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <Core/MW/Publisher.hpp>
#include <Core/MW/Subscriber.hpp>
#include <Core/MW/CoreNode.hpp>
#include <Core/MW/CoreSensor.hpp>
#include <Core/MW/Callback.hpp>

#include <Configuration.hpp>

#include <sensor_msgs/RPY_f32.hpp>
#include <madgwick/MadgwickConfiguration.hpp>

namespace madgwick {
   class Madgwick:
      public Core::MW::CoreNode
   {
public:
      Madgwick(
         const char*                name,
         Core::MW::Thread::Priority priority = Core::MW::Thread::PriorityEnum::NORMAL
      );
      virtual
      ~Madgwick();

public:
      MadgwickConfiguration configuration;

private:
      Configuration::L3GD20H_GYRO_DATATYPE _gyroData;
      Configuration::LSM303D_ACC_DATATYPE  _accData;
      Configuration::LSM303D_MAG_DATATYPE  _magData;

private:
      bool
      onPrepareMW();

      bool
      onLoop();

      static bool
      gyroCallback(
         const Configuration::L3GD20H_GYRO_DATATYPE& msg,
         Core::MW::Node*                             node
      );

      static bool
      accCallback(
         const Configuration::LSM303D_ACC_DATATYPE& msg,
         Core::MW::Node*                            node
      );

      static bool
      magCallback(
         const Configuration::LSM303D_MAG_DATATYPE& msg,
         Core::MW::Node*                            node
      );


private:
      Core::MW::Subscriber<Configuration::L3GD20H_GYRO_DATATYPE, 2> _subscriberGyro;
      Core::MW::Subscriber<Configuration::LSM303D_ACC_DATATYPE, 2>  _subscriberAcc;
      Core::MW::Subscriber<Configuration::LSM303D_MAG_DATATYPE, 2>  _subscriberMag;
      Core::MW::Publisher<sensor_msgs::RPY_f32> _publisher;
   };
}
