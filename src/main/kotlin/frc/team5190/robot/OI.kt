@file:Suppress("unused")

/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.buttons.JoystickButton
import frc.team5190.robot.elevator.AutoElevatorCommand
import frc.team5190.robot.elevator.ElevatorPosition

/**
 * GameCube Bongo Controller object
 */

object Bongo : Joystick(0) {
    fun getLeftBongoSpeed() = when {
        this.getRawButton(4) -> 0.5
        this.getRawButton(2) -> -0.5
        else -> 0.0
    }

    fun getRightBongoSpeed() = when {
        this.getRawButton(3) -> 0.5
        this.getRawButton(1) -> -0.5
        else -> 0.0
    }
}
/**
 * Xbox Controller object
 */
object MainXbox : XboxController(0) {
    init {
        JoystickButton(this, 1).whenPressed(AutoElevatorCommand(ElevatorPosition.SCALE)) // A button
        JoystickButton(this, 3).whenPressed(AutoElevatorCommand(ElevatorPosition.SWITCH)) // X button
//        JoystickButton(this, 2).whenPressed(ResetElevatorCommand()) // B button
    }
}

fun XboxController.getLeftX() = getX(GenericHID.Hand.kLeft)
fun XboxController.getLeftY() = getY(GenericHID.Hand.kLeft)

fun XboxController.getRightX() = getX(GenericHID.Hand.kRight)
fun XboxController.getRightY() = getY(GenericHID.Hand.kRight)