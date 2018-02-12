/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.sensors

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI

/**
 * Creates a NavX singleton object
 */
object NavX : AHRS(SPI.Port.kMXP){

    init {
        zeroYaw()
    }
}