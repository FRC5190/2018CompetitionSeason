/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

@file:Suppress("unused")

package frc.team5190.robot

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.buttons.JoystickButton
import frc.team5190.robot.climb.DeployHookCommand

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
object MainXbox : XboxController(0)

fun XboxController.getLeftX() = getX(GenericHID.Hand.kLeft)
fun XboxController.getLeftY() = getY(GenericHID.Hand.kLeft)

fun XboxController.getRightX() = getX(GenericHID.Hand.kRight)
fun XboxController.getRightY() = getY(GenericHID.Hand.kRight)

fun XboxController.getTriggerPressed(hand: GenericHID.Hand, amount: Double = 0.5) = getTriggerAxis(hand) > amount