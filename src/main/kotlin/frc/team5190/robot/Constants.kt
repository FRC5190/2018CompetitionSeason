package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX

enum class MotorIds(val id: Int) {
    FRONT_LEFT_VAL(1),
    FRONT_RIGHT_VAL(3),
    REAR_LEFT_VAL(2),
    REAR_RIGHT_VAL(4);

    val talon by lazy { WPI_TalonSRX(id) }
}

object Hardware {
    const val SENSOR_UNITS_PER_ROTATION = 360
    const val WHEEL_RADIUS = 2

    const val MAX_RPM = 1125
    const val HIGH_GEAR_MAX = 18.3
}

fun talonListOf(vararg elements: MotorIds): List<WPI_TalonSRX> = listOf(*elements).map { it.talon }
