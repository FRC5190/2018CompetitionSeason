/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.util

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.SensorBase

/**
 * Contains Motor IDs.
 */
object MotorIDs {
    const val FRONT_LEFT = 5
    const val FRONT_RIGHT = 3
    const val REAR_LEFT = 2
    const val REAR_RIGHT = 1
  
    const val ELEVATOR_MASTER = 20
    const val ELEVATOR_SLAVE = 21

    const val ARM = 25

    const val INTAKE_LEFT = 10
    const val INTAKE_RIGHT = 11
}

object SolenoidIDs {
    private const val PCM = 42

    const val INTAKE = 0

    init {
        SensorBase.setDefaultSolenoidModule(PCM)
    }
}

/**
 * Contains constants related to hardware.
 */
object Hardware {
    const val NATIVE_UNITS_PER_ROTATION = 1440
    const val WHEEL_RADIUS = 2.0

    const val MAX_RPM = 925
    const val MAX_NATIVE_UNITS_PER_100_MS = 2220         // rpm * sensorUnitsPerRotation / 600
}

/**
 * Scales the output depending on the ControlMode.
 */
fun ControlMode.scale(): Double {
    return when (this) {
        ControlMode.PercentOutput -> 1.0
        ControlMode.Velocity -> Hardware.MAX_NATIVE_UNITS_PER_100_MS.toDouble()
        else -> TODO("Scaling for $name is not supported!")
    }
}
