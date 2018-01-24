package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand

class ResetArmCommand : TimedCommand(2.0) {
    init {
        requires(ArmSubsystem)
    }

    override fun execute() {
        ArmSubsystem.set(ControlMode.PercentOutput, 0.2)
    }

    override fun end() {
        ArmSubsystem.set(ControlMode.Position, ArmSubsystem.currentPosition.toDouble())
        ArmSubsystem.resetEncoders()
    }
}
