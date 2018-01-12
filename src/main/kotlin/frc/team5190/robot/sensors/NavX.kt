package frc.team5190.robot.sensors

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.I2C

object NavX : AHRS(I2C.Port.kMXP){

    init {
        zeroYaw()
    }

}