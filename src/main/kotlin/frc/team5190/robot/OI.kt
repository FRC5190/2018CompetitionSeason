@file:Suppress("unused")

/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.buttons.JoystickButton
import frc.team5190.robot.elevator.*

object Bongos : Joystick(0) {
    fun getLeftBongoSpeed() = when {
        this.getRawButton(4) -> 0.6
        this.getRawButton(2) -> -0.6
        else -> 0.0
    }

    fun getRightBongoSpeed() = when {
        this.getRawButton(3) -> 0.6
        this.getRawButton(1) -> -0.6
        else -> 0.0
    }
}


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