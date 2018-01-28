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
    private const val PCM = 41

    const val DRIVE_PCM = 3
    const val INTAKE = 2

    init {
        SensorBase.setDefaultSolenoidModule(PCM)
    }
}

/**
 * Contains constants related to hardware.
 */
object Hardware {
    const val NATIVE_UNITS_PER_ROTATION = 1440
    const val WHEEL_RADIUS = 3.0

    const val MAX_RPM_HIGH = 925
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
