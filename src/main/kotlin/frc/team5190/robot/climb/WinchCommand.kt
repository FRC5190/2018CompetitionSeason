package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.Controls


class WinchCommand : Command() {

    init {
        requires(ClimbSubsystem)
    }

    override fun execute() = Controls.winchSubsystem()

    override fun isFinished() = false
}

class IdleClimbCommand : Command() {

    init {
        requires(ClimbSubsystem)
    }

    override fun initialize() {
        ClimbSubsystem.set(ControlMode.PercentOutput, 0.0)
    }

    override fun isFinished() = false
}