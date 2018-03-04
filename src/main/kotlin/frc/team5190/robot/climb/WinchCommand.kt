package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox


class WinchCommand : Command() {

    init {
        requires(ClimbSubsystem)
    }

    override fun execute() {
        if (!MainXbox.getBumper(GenericHID.Hand.kLeft)) ClimbSubsystem.frontWinchMotor.set(ControlMode.PercentOutput, MainXbox.getTriggerAxis(GenericHID.Hand.kLeft))
        if (!MainXbox.getBumper(GenericHID.Hand.kRight)) ClimbSubsystem.backWinchMotor.set(ControlMode.PercentOutput, MainXbox.getTriggerAxis(GenericHID.Hand.kRight))

        if (MainXbox.getBumper(GenericHID.Hand.kLeft)) ClimbSubsystem.frontWinchMotor.set(ControlMode.PercentOutput, -0.3)
        if (MainXbox.getBumper(GenericHID.Hand.kRight)) ClimbSubsystem.backWinchMotor.set(ControlMode.PercentOutput, -0.3)
    }

    override fun isFinished() = false
}