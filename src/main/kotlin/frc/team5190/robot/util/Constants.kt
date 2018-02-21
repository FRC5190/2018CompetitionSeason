/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.util

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

    const val BACK_WINCH_MASTER = 0 // 10
    const val BACK_WINCH_SLAVE  = 0 // 11
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

    const val P_HIGH = 2.0
    const val I_HIGH = 0.0
    const val D_HIGH = 20.0

    const val PID_SLOT_HIGH = 0

    const val MAX_RPM_LOW = 925
    const val MAX_STU_LOW = 2220

    const val P_LOW = 0.7
    const val I_LOW = 0.0
    const val D_LOW = 0.0

    const val PID_SLOT_LOW = 1

    const val MOTION_MAGIC_CRUISE = 7.0
    const val MOTION_MAGIC_ACCEL = 4.5

    const val IS_RACE_ROBOT = true                 // true for falcon heavy, false for falcon 9
}

object ElevatorConstants {
    // robot settings for elevator
    const val SENSOR_UNITS_PER_ROTATION = 1440      // 4096 for falcon heacy, 1440 for falcon 9

    const val P = 0.5
    const val I = 0.0
    const val D = 0.0
    const val PID_SLOT = 0

    // current limiting
    const val LOW_PEAK = 5
    const val HIGH_PEAK = 30
    const val DUR = 1000
    const val LIMITING_REDUCTION_FACTOR = 0.3

    // nominal and peak currents
    const val NOMINAL_OUT = 0.0
    const val PEAK_OUT = 0.75
    const val TOLERANCE_INCHES = 0.25

    // motion magic
    const val MOTION_VELOCITY = 1000000000
    const val MOTION_ACCELERATION_INCHES = 50.0
}

object ArmConstants {
    // robot settings for arm
    const val INVERTED = true                  // true for falcon heavy, false for falcon  9
    const val SENSOR_PHASE = true              // true for falcon heavy, false for falcon 9

    const val P = 2.5
    const val I = 0.0
    const val D = 0.0
    const val PID_SLOT = 0

    const val DOWN_TICKS = 1400                 // 1470 for falcon heavy, 1840 for falcon 9

    // current limiting
    const val LOW_PEAK = 5
    const val HIGH_PEAK = 30
    const val DUR = 1000
    const val LIMITING_REDUCTION_FACTOR = 0.3

    // nominal and peak currents
    const val NOMINAL_OUT = 0.0
    const val PEAK_OUT = 0.4
    const val TOLERANCE = 0

    // motion magic
    const val MOTION_VELOCITY = 1000000
    const val MOTION_ACCELERATION = 350
}

object IntakeConstants {
    const val DEFAULT_SPEED = 0.65
    const val AMP_THRESHOLD = 15
}
