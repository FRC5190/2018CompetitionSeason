/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.util

import com.ctre.phoenix.motorcontrol.ControlMode

/**
 * Contains Motor IDs.
 */
object MotorIDs {
    const val FRONT_LEFT = 6
    const val FRONT_RIGHT = 5
    const val REAR_LEFT = 3
    const val REAR_RIGHT = 4
}

/**
 * Contains constants related to hardware.
 */
object Hardware {
    const val NATIVE_UNITS_PER_ROTATION = 1440
    const val WHEEL_RADIUS = 3.0

    const val MAX_RPM = 1065
    val MAX_NATIVE_UNITS_PER_100_MS = Maths.rpmToNativeUnitsPer100Ms(MAX_RPM.toDouble(), WHEEL_RADIUS)
}

/**
 * Scales the output depending on the ControlMode.
 */
fun ControlMode.scale(): Double {
    return when (this) {
        ControlMode.PercentOutput -> 1.0
        ControlMode.Velocity -> Hardware.MAX_NATIVE_UNITS_PER_100_MS
        else -> TODO("Scaling for $name is not supported!")
    }
}
