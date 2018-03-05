package frc.team5190.robot.climb

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.Controls


class WinchCommand : Command() {

    init {
        requires(ClimbSubsystem)
    }

    override fun execute() = Controls.winchSubsystem()

    override fun isFinished() = false
}