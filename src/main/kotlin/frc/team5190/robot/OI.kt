@file:Suppress("unused")

/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.buttons.JoystickButton
import frc.team5190.robot.auto.AutoCommand
import frc.team5190.robot.auto.AutoHelper

/**
 * Xbox Controller object
 */
object MainXbox : XboxController(0) {
    init {
        JoystickButton(this, 1).whenPressed(AutoCommand(AutoHelper.RIGHTS_LEFT))
    }
}

fun XboxController.getLeftX() = getX(GenericHID.Hand.kLeft)
fun XboxController.getLeftY() = getY(GenericHID.Hand.kLeft)

fun XboxController.getRightX() = getX(GenericHID.Hand.kRight)
fun XboxController.getRightY() = getY(GenericHID.Hand.kRight)