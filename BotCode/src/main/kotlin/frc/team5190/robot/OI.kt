package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController

object MainXbox : XboxController(0)

fun XboxController.leftY() = getY(GenericHID.Hand.kLeft)
fun XboxController.leftX() = getX(GenericHID.Hand.kLeft)

fun XboxController.rightY() = getY(GenericHID.Hand.kRight)
fun XboxController.rightX() = getX(GenericHID.Hand.kRight)