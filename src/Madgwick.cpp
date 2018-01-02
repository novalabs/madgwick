/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <core/madgwick/Madgwick.hpp>
#include <core/sensor_msgs/RPY_f32.hpp>
#include <core/L3GD20H_driver/L3GD20H.hpp>
#include <core/LSM303D_driver/LSM303D.hpp>

#include "core/madgwick/madgwick.h"

namespace core {
namespace madgwick {
Madgwick::Madgwick(
    const char*                name,
    core::os::Thread::Priority priority
) :
    CoreNode::CoreNode(name, priority),
    CoreConfigurable<core::madgwick::MadgwickConfiguration>::CoreConfigurable(name)
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

    this->subscribe(_subscriberGyro, configuration().topicGyro);
    this->subscribe(_subscriberAcc, configuration().topicAcc);
    this->subscribe(_subscriberMag, configuration().topicMag);

    this->advertise(_publisher, configuration().topic);

    return true;
}

bool
Madgwick::onLoop()
{
//      core::os::Time t = core::os::Time::now();

    while (mustLoop()) {
        // Override the CoreNode loop...
        if (core::os::Thread::should_terminate()) {
            teardown();
        } else {
            /*
               if (!this->spin(Configuration::SUBSCRIBER_SPIN_TIME)) {
               Module::led.toggle();
               } else {
             */
            core::common_msgs::Vector3_i16 acc_msg;
            core::common_msgs::Vector3_i16 gyro_msg;

            systime_t time;
            time = chVTGetSystemTime();

            Module::acc.get(acc_msg);
            Module::gyro.get(gyro_msg);

            // --- LOOP BEGIN -----------------------------------------------------
            MadgwickAHRSupdateIMU(core::utils::math::conversions::Deg2Rad<float>(gyro_msg.y / 57.143f),
                                  core::utils::math::conversions::Deg2Rad<float>(-gyro_msg.x / 57.143f),
                                  core::utils::math::conversions::Deg2Rad<float>(gyro_msg.z / 57.143f),
                                  acc_msg.x / 1000.0,
                                  acc_msg.y / 1000.0,
                                  acc_msg.z / 1000.0);

            attitude_t attitude;

            getMadAttitude(&attitude);

            sensor_msgs::RPY_f32* msgp;

            if (_publisher.alloc(msgp)) {
                msgp->roll  = core::utils::math::conversions::Rad2Deg<float>(attitude.roll);
                msgp->pitch = core::utils::math::conversions::Rad2Deg<float>(attitude.pitch);
                msgp->yaw   = core::utils::math::conversions::Rad2Deg<float>(attitude.yaw);

                _publisher.publish(*msgp);
            }

            /*
               }
             */
            /* Broken...
               t = t + core::os::Time::ms((core::os::Time::Type)(1000.0f / configuration.frequency));
               //core::os::Thread::sleep_until(t);
               core::os::Thread::sleep(core::os::Time::ms(20));
             */

            time += MS2ST(20);
            chThdSleepUntil(time);

            // --- LOOP END -------------------------------------------------------
        }
    }

    return true;
}       // Madgwick::onLoop

bool
Madgwick::gyroCallback(
    const ModuleConfiguration::L3GD20H_GYRO_DATATYPE& msg,
    void*                                             context
)
{
    Madgwick* tmp = static_cast<Madgwick*>(context);

    tmp->_gyroData = msg;

    return true;
}

bool
Madgwick::accCallback(
    const ModuleConfiguration::LSM303D_ACC_DATATYPE& msg,
    void*                                            context
)
{
    Madgwick* tmp = static_cast<Madgwick*>(context);

    tmp->_accData = msg;

    return true;
}

bool
Madgwick::magCallback(
    const ModuleConfiguration::LSM303D_MAG_DATATYPE& msg,
    void*                                            context
)
{
    Madgwick* tmp = static_cast<Madgwick*>(context);

    tmp->_magData = msg;

    return true;
}
}
}
