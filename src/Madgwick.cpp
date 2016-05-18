/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <madgwick/Madgwick.hpp>
#include <Core/Utils/Math/Constants.hpp>
#include <Core/Utils/Math/Conversions.hpp>
#include <sensor_msgs/RPY_f32.hpp>
#include <sensors/L3GD20H.hpp>
#include <sensors/LSM303D.hpp>

#include "madgwick/madgwick.h"


namespace madgwick {
   Madgwick::Madgwick(
      const char*                    name,
      Core::MW::Thread::PriorityEnum priority
   ) :
      CoreNode::CoreNode(name, priority)
   {
      _workingAreaSize = 512;
   }

   Madgwick::~Madgwick()
   {
      teardown();
   }

   bool
   Madgwick::onPrepareMW()
   {
      _subscriberGyro.set_callback(Madgwick::gyroCallback);
      _subscriberAcc.set_callback(Madgwick::accCallback);
      _subscriberMag.set_callback(Madgwick::magCallback);

      this->subscribe(_subscriberGyro, configuration.topicGyro);
      this->subscribe(_subscriberAcc, configuration.topicAcc);
      this->subscribe(_subscriberMag, configuration.topicMag);

      this->advertise(_publisher, configuration.topic);

      return true;
   }

   bool
   Madgwick::onLoop()
   {
      Core::MW::Time t = Core::MW::Time::now();

      while (mustLoop()) {
         // Override the CoreNode loop...
         if (Core::MW::Thread::should_terminate()) {
            teardown();
         } else {
            /*
               if (!this->spin(Configuration::SUBSCRIBER_SPIN_TIME)) {
               Module::led.toggle();
               } else {
             */
            common_msgs::Vector3_i16 acc_msg;
            common_msgs::Vector3_i16 gyro_msg;

            systime_t time;
            time = chVTGetSystemTime();

            Module::acc.get(acc_msg);
            Module::gyro.get(gyro_msg);

            // --- LOOP BEGIN -----------------------------------------------------
            MadgwickAHRSupdateIMU(Core::Utils::Math::Conversions::Deg2Rad<float>(gyro_msg.y / 57.143f),
                                  Core::Utils::Math::Conversions::Deg2Rad<float>(-gyro_msg.x / 57.143f),
                                  Core::Utils::Math::Conversions::Deg2Rad<float>(gyro_msg.z / 57.143f),
                                  acc_msg.x / 1000.0,
                                  acc_msg.y / 1000.0,
                                  acc_msg.z / 1000.0);

            attitude_t attitude;

            getMadAttitude(&attitude);

            sensor_msgs::RPY_f32* msgp;

            if (_publisher.alloc(msgp)) {
               msgp->roll  = Core::Utils::Math::Conversions::Rad2Deg<float>(attitude.roll);
               msgp->pitch = Core::Utils::Math::Conversions::Rad2Deg<float>(attitude.pitch);
               msgp->yaw   = Core::Utils::Math::Conversions::Rad2Deg<float>(attitude.yaw);

               _publisher.publish(*msgp);
            }

            /*
               }
             */
            /* Broken...
               t = t + Core::MW::Time::ms((Core::MW::Time::Type)(1000.0f / configuration.frequency));
               //Core::MW::Thread::sleep_until(t);
               Core::MW::Thread::sleep(Core::MW::Time::ms(20));
             */

            time += MS2ST(20);
            chThdSleepUntil(time);

            // --- LOOP END -------------------------------------------------------
         }
      }

      return true;
   } // Madgwick::onLoop

   bool
   Madgwick::gyroCallback(
      const Configuration::L3GD20H_GYRO_DATATYPE& msg,
      Core::MW::Node*                             node
   )
   {
      Madgwick* tmp = static_cast<Madgwick*>(node);

      tmp->_gyroData = msg;

      return true;
   }

   bool
   Madgwick::accCallback(
      const Configuration::LSM303D_ACC_DATATYPE& msg,
      Node*                                      node
   )
   {
      Madgwick* tmp = static_cast<Madgwick*>(node);

      tmp->_accData = msg;

      return true;
   }

   bool
   Madgwick::magCallback(
      const Configuration::LSM303D_MAG_DATATYPE& msg,
      Node*                                      node
   )
   {
      Madgwick* tmp = static_cast<Madgwick*>(node);

      tmp->_magData = msg;

      return true;
   }
}
