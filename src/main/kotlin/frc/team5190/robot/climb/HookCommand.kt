package frc.team5190.robot.climb

import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.climb.ClimbSubsystem.hookSolenoid


class DeployHookCommand : TimedCommand(0.5) {
    init {
        requires(ClimbSubsystem)
    }

    override fun initialize() {
        hookSolenoid.set(true)
    }
}

class DefaultHookCommand : Command() {
    init {
        requires(ClimbSubsystem)
    }

    override fun initialize() {
        hookSolenoid.set(false)
    }

    override fun isFinished() = false
}

