@file:Suppress("unused")

/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.buttons.JoystickButton
import frc.team5190.robot.elevator.*

/**
 * Xbox Controller object
 */
object MainXbox : XboxController(0) {
    init {
        JoystickButton(this, 1).whenPressed(AutoElevatorCommand(ElevatorPosition.SCALE))
        JoystickButton(this, 3).whenPressed(AutoElevatorCommand(ElevatorPosition.SWITCH))
        JoystickButton(this, 2).whenPressed(ResetElevatorCommand())
    }
}

fun XboxController.getLeftX() = getX(GenericHID.Hand.kLeft)
fun XboxController.getLeftY() = getY(GenericHID.Hand.kLeft)

fun XboxController.getRightX() = getX(GenericHID.Hand.kRight)
fun XboxController.getRightY() = getY(GenericHID.Hand.kRight)