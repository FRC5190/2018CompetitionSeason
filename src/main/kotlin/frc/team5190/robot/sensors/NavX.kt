/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.sensors

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.I2C

/**
 * Creates a NavX singleton object
 */
object NavX : AHRS(I2C.Port.kMXP) {

    init {
        zeroYaw()
    }

    private val modifiedPitch: Float
        get () = ((this.pitch - resetPitch + 180) % 360) - 180

    var resetPitch = this.modifiedPitch

    override fun getAngle() = -modifiedPitch.toDouble()

    override fun pidGet() = angle

    override fun reset() {
        resetPitch = this.pitch
    }
}