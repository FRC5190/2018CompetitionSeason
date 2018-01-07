package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController

object OI {
    val xbox = XboxController(0)

    val yLeft: Double
        get() = xbox.getY(GenericHID.Hand.kLeft)

    val xLeft: Double
        get() = xbox.getX(GenericHID.Hand.kLeft)
}
