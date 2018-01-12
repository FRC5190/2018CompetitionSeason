package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController

object MainXbox : XboxController(0)

fun XboxController.getLeftX() = getX(GenericHID.Hand.kLeft)
fun XboxController.getLeftY() = getY(GenericHID.Hand.kLeft)

fun XboxController.getRightX() = getX(GenericHID.Hand.kRight)
fun XboxController.getRightY() = getY(GenericHID.Hand.kRight)