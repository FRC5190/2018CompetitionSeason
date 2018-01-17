@file:Suppress("unused")

package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.buttons.JoystickButton
import frc.team5190.robot.auto.NAVCommand
import frc.team5190.robot.auto.NAVHelper

object MainXbox : XboxController(0) {
    init {
        JoystickButton(this, 1).whenPressed(NAVCommand(Robot.autoChooser.selected ?: NAVHelper.RIGHTS_LEFT))
    }
}

fun XboxController.getLeftX() = getX(GenericHID.Hand.kLeft)
fun XboxController.getLeftY() = getY(GenericHID.Hand.kLeft)

fun XboxController.getRightX() = getX(GenericHID.Hand.kRight)
fun XboxController.getRightY() = getY(GenericHID.Hand.kRight)