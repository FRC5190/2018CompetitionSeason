/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.util

import com.ctre.phoenix.motorcontrol.ControlMode
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.drive.Gear

/**
 * Contains Motor IDs.
 */
object MotorIDs {
    const val FRONT_LEFT = 1
    const val FRONT_RIGHT = 3
    const val REAR_LEFT = 2
    const val REAR_RIGHT = 4

    const val ELEVATOR_MASTER = 5
    const val ELEVATOR_SLAVE = 6

    const val ARM = 8

    const val INTAKE_LEFT = 7
    const val INTAKE_RIGHT = 9
}

object SolenoidIDs {
    const val PCM = 41

    const val DRIVE = 3
    const val INTAKE = 2
}

object DriveConstants {
    const val SENSOR_UNITS_PER_ROTATION = 1440
    const val WHEEL_RADIUS = 3.0

    const val MAX_RPM_HIGH = 925
    const val MAX_STU_HIGH = 2220

    const val P_HIGH = 0.7
    const val I_HIGH = 0.0
    const val D_HIGH = 0.0

    const val PID_SLOT_HIGH = 0

    const val MAX_RPM_LOW = 925
    const val MAX_STU_LOW = 2220

    const val P_LOW = 0.7
    const val I_LOW = 0.0
    const val D_LOW = 0.0

    const val PID_SLOT_LOW = 1

    const val IS_RACE_ROBOT = false
}

object ElevatorConstants {
    const val SENSOR_UNITS_PER_ROTATION = 1440

    const val PEAK_WATTAGE = 100
    const val PEAK_DURATION = 1000

    const val NOMINAL_OUT = 0.0
    const val PEAK_OUT = 0.7

    const val P = 0.8
    const val I = 0.0
    const val D = 0.0

    const val PID_SLOT = 0

    const val TOLERANCE_INCHES = 0.25

    const val MOTION_VELOCITY = 1000000000
    const val MOTION_ACCELERATION_INCHES = 80.0

    const val LIMITING_REDUCTION_FACTOR = 0.3
}

object ArmConstants {
    const val PEAK_WATTAGE = 15
    const val PEAK_DURATION = 1000

    const val P = 1.5
    const val I = 0.0
    const val D = 0.0

    const val PID_SLOT = 0

    const val NOMINAL_OUT = 0.0
    const val PEAK_OUT = 0.7

    const val TOLERANCE = 0

    const val MOTION_VELOCITY = 1000000
    const val MOTION_ACCELERATION = 400

    const val LIMITING_REDUCTION_FACTOR = 0.5
}

object IntakeConstants {
    const val PEAK_WATTAGE = 100
    const val PEAK_DURATION = 1000

    const val DEFAULT_SPEED = 0.8

    const val LIMITING_REDUCTION_FACTOR = 0.3
}

/**
 * Scales the output depending on the ControlMode.
 */
fun ControlMode.scale(): Double {
    return when (this) {
        ControlMode.PercentOutput -> 1.0
        ControlMode.Velocity ->
            if (DriveSubsystem.falconDrive.gear == Gear.LOW) DriveConstants.MAX_STU_LOW.toDouble()
            else DriveConstants.MAX_STU_HIGH.toDouble()
        else -> TODO("Not supported.")
    }
}
