/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */


package frc.team5190.robot.util

// Motor IDs
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

    const val WINCH_MASTER = 10
    const val WINCH_SLAVE = 59

}

// PWM and Analog Channel IDs
object ChannelIDs {
    const val LEFT_CUBE_SENSOR = 2
    const val RIGHT_CUBE_SENSOR = 3

    const val LIDAR_SERVO = 0
}

// Solenoid IDs
object SolenoidIDs {
    const val PCM = 41

    const val DRIVE = 3
    const val INTAKE = 2
}

// Drive Constants
object DriveConstants {
    const val SENSOR_UNITS_PER_ROTATION = 1440
    const val WHEEL_RADIUS = 2.92
    const val DRIVE_BASE_WIDTH = 29.5

    const val MAX_RPM_HIGH = 925
    const val MAX_STU_HIGH = 2220

    const val FOLLOW_BETA = 0.30
    const val FOLLOW_ZETA = 0.85

    const val P_HIGH = 2.0
    const val I_HIGH = 0.0
    const val D_HIGH = 0.0
    const val V_HIGH = 0.656
    const val A_HIGH = 0.030
    const val S_HIGH = 0.050

    const val PID_SLOT_HIGH = 0

    const val MAX_RPM_LOW = 925
    const val MAX_STU_LOW = 2220

    const val P_LOW = 0.7
    const val I_LOW = 0.0
    const val D_LOW = 0.0

    const val PID_SLOT_LOW = 1

    const val TURN_P = 0.08
    const val TURN_I = 0.002
    const val TURN_D = 0.1

    const val MOTION_MAGIC_CRUISE = 7.0
    const val MOTION_MAGIC_ACCEL = 4.5

    const val MOTION_DT = 0.02
    const val AUTO_SHIFT_LOW_THRESHOLD = 2.0
    const val AUTO_SHIFT_HIGH_THRESHOLD = 5.0

    const val TRACK_WIDTH = 2.6
}

// Elevator Constants
object ElevatorConstants {
    const val SENSOR_UNITS_PER_ROTATION = 1440

    const val P = 0.3
    const val I = 0.0
    const val D = 0.0
    const val F = 0.0


    const val PID_SLOT = 0

    const val LOW_PEAK = 5
    const val HIGH_PEAK = 30
    const val DUR = 1000

    const val NOMINAL_OUT = 0.0
    const val IDLE_PEAK_OUT = 0.2
    const val ACTIVE_PEAK_OUT = 1.0
    const val TOLERANCE_INCHES = 0.25

    const val SOFT_LIMIT_FWD = 22500

    const val MOTION_VELOCITY = 72.0
    const val MOTION_ACCELERATION_INCHES = 90.0
}

// Arm Constants
object ArmConstants {
    const val INVERTED = true
    const val SENSOR_PHASE = false

    const val P = 4.5
    const val I = 0.0
    const val D = 0.0
    const val F = 0.0

    const val PID_SLOT = 0

    const val DOWN_TICKS = -795

    const val NOMINAL_OUT = 0.0
    const val PEAK_OUT = 1.00
    const val TOLERANCE = 0

    const val MOTION_VELOCITY = 1000000
    const val MOTION_ACCELERATION = 350
}

// Intake Constants
object IntakeConstants {
    const val DEFAULT_IN_SPEED = 1.0
    const val DEFAULT_OUT_SPEED = 0.65
}

// Climbing Constants
object ClimbConstants {
    const val SCALE_POS = 47600
    const val CLIMB_POS = 0

    const val PEAK_OUTPUT = 1.0
}

// Universal CTRE Timeout
const val TIMEOUT = 20


