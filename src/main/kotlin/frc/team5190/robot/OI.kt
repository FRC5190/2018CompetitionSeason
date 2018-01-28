@file:Suppress("unused")

/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot

import edu.wpi.first.wpilibj.*

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
//        JoystickButton(this, 1).whenPressed(ResetElevatorCommand())
//        JoystickButton(this, 3).whenPressed(AutoElevatorCommand(ElevatorPosition.SWITCH))
    }
}

fun XboxController.getLeftX() = getX(GenericHID.Hand.kLeft)
fun XboxController.getLeftY() = getY(GenericHID.Hand.kLeft)

fun XboxController.getRightX() = getX(GenericHID.Hand.kRight)
fun XboxController.getRightY() = getY(GenericHID.Hand.kRight)