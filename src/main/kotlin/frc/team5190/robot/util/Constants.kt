package frc.team5190.robot.util

object MotorIDs {
    val FRONT_LEFT = 1
    val FRONT_RIGHT = 3
    val REAR_LEFT = 2
    val REAR_RIGHT = 4
}

object Hardware {
    val NATIVE_UNITS_PER_ROTATION = 1440
    val WHEEL_RADIUS = 3.75 / 2.0

    val MAX_RPM = 1050

    val MAX_LEFT_RPM = 1065
    val MAX_RIGHT_RPM = 1065

    val MAX_NATIVE_UNITS_PER_100_MS = Maths.rpmToNativeUnitsPer100Ms(MAX_RPM.toDouble(), WHEEL_RADIUS.toDouble())
}
