/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.util

import com.ctre.phoenix.motorcontrol.ControlMode

object MotorIDs {
    val FRONT_LEFT = 1
    val FRONT_RIGHT = 3
    val REAR_LEFT = 2
    val REAR_RIGHT = 4
}

object Hardware {
    val NATIVE_UNITS_PER_ROTATION = 1440
    val WHEEL_RADIUS = 3.75 / 2.0

    val MAX_RPM = 1065

    val MAX_NATIVE_UNITS_PER_100_MS = Maths.rpmToNativeUnitsPer100Ms(MAX_RPM.toDouble(), WHEEL_RADIUS)
}

fun ControlMode.scale(): Double {
    return when (this) {
        ControlMode.PercentOutput -> 1.0
        ControlMode.Velocity -> Hardware.MAX_NATIVE_UNITS_PER_100_MS
        else -> TODO("Scaling for $name is not supported!")
    }
}
