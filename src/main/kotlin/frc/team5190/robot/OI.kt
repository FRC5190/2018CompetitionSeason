package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.buttons.JoystickButton

class OI {
    companion object Xbox {
        
        val xbox = XboxController(0)

        private var buttonA = JoystickButton(xbox, 1)

        init {
            buttonA.whenPressed(frc.team5190.robot.Reset())

        }

        fun getLeftY() = xbox.getY(GenericHID.Hand.kLeft)
        fun getRightY() = xbox.getY(GenericHID.Hand.kRight)

    }
}