package frc.team5190.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController

class OI {
    companion object {
        private val xbox = XboxController(0)

        fun getY(hand : GenericHID.Hand) : Double{
            return xbox.getY(hand)
        }
    }
}