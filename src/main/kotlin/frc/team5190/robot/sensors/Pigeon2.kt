package frc.team5190.robot.sensors

import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.sensors.PigeonIMU
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.I2C
import frc.team5190.robot.util.TIMEOUT
import jaci.pathfinder.Pathfinder

object Pigeon2 : AHRS(I2C.Port.kMXP) {

    init {
        reset()
    }

    var angleOffset = 0.0

    val correctedAngle: Double
        get() = Pathfinder.boundHalfDegrees(-angle + angleOffset)

    fun update() {

    }
}
