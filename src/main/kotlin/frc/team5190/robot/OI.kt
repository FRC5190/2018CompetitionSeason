package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController

object OI {
    val xbox = XboxController(0)

    fun getYLeft(): Double {
        return xbox.getY(GenericHID.Hand.kLeft)
    }

    fun getXLeft(): Double {
        return xbox.getX(GenericHID.Hand.kLeft)
    }
}
