package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox


class ManualArmCommand : Command() {

    init {
        requires(ArmSubsystem)
    }

    override fun execute() {
        when {
            MainXbox.yButton -> ArmSubsystem.set(ControlMode.PercentOutput, 0.4)
            MainXbox.bButton -> ArmSubsystem.set(ControlMode.PercentOutput, -0.4)

            MainXbox.yButtonReleased -> ArmSubsystem.set(ControlMode.PercentOutput, 0.0)
            MainXbox.bButtonReleased -> ArmSubsystem.set(ControlMode.PercentOutput, 0.0)
        }
    }

    override fun isFinished() = false
}
