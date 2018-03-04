package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.TimedCommand

class DeployHookCommand : TimedCommand(0.5) {
    init {
        requires(ClimbSubsystem)
    }

    override fun initialize() {
        ClimbSubsystem.hookSolenoid.set(true)
    }
}

class IdleClimbCommand : Command() {
    init {
        requires(ClimbSubsystem)
    }

    override fun initialize() {
        ClimbSubsystem.hookSolenoid.set(false)

        ClimbSubsystem.backWinchMotor.set(ControlMode.PercentOutput, 0.0)
        ClimbSubsystem.frontWinchMotor.set(ControlMode.PercentOutput, 0.0)
    }

    override fun isFinished() = false
}

