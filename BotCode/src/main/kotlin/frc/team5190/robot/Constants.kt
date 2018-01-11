package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX

enum class MotorIds(val id: Int) {
    FRONT_LEFT_VAL(1),
    FRONT_RIGHT_VAL(3),
    REAR_LEFT_VAL(2),
    REAR_RIGHT_VAL(4);

    val talon by lazy { WPI_TalonSRX(id) }
}

fun talonListOf(vararg elements: MotorIds): List<WPI_TalonSRX> = listOf(*elements).map { it.talon }
