package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.util.MotorIDs
import frc.team5190.robot.util.configPeakOutput


class ArmCommand : Command() {

    init {
        requires(ArmSubsystem)
    }

    override fun execute() {
        when {
            MainXbox.getTriggerAxis(GenericHID.Hand.kLeft) > 0 -> ArmSubsystem.armMotor.set(ControlMode.PercentOutput, MainXbox.getTriggerAxis(GenericHID.Hand.kLeft))
            MainXbox.getTriggerAxis(GenericHID.Hand.kRight) > 0 -> ArmSubsystem.armMotor.set(ControlMode.PercentOutput, -MainXbox.getTriggerAxis(GenericHID.Hand.kRight))
            else -> ArmSubsystem.armMotor.set(ControlMode.PercentOutput, 0.0)
        }
    }

    override fun isFinished() = false
}


object ArmSubsystem : Subsystem() {

    val armMotor = TalonSRX(MotorIDs.ARM)

    init {
        armMotor.configPeakOutput(0.4, -0.4, 10)
    }

    override fun initDefaultCommand() {
        this.defaultCommand = ArmCommand()
    }
}