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
    const val DRIVE_FRONT_LEFT = 1
    const val DRIVE_FRONT_RIGHT = 3
    const val DRIVE_REAR_LEFT = 2
    const val DRIVE_REAR_RIGHT = 4

    const val ARM = 5

    const val INTAKE_LEFT = 10
    const val INTAKE_RIGHT = 11
}

/**
 * Contains constants related to hardware.
 */
object Hardware {
    val NATIVE_UNITS_PER_ROTATION = 1440
    val WHEEL_RADIUS = 3.75 / 2.0

    val MAX_RPM = 1065
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
