package frc.team5190.robot.climb

import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.MainXbox
import frc.team5190.robot.climb.HookSubsystem.hookSolenoid
import frc.team5190.robot.util.SolenoidIDs

object HookSubsystem : Subsystem() {

    val hookSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.HOOK)
    override fun initDefaultCommand() {
        this.defaultCommand = DefaultHookCommand()
    }

    override fun periodic() {
        if (MainXbox.backButton) DeployHookCommand().start()
    }

}

class DeployHookCommand : TimedCommand(0.5) {

    init {
        requires(HookSubsystem)
    }

    override fun initialize() {
        hookSolenoid.set(true)
    }

    override fun isFinished() = isTimedOut
}

class DefaultHookCommand : Command() {

    init {
        requires(HookSubsystem)
    }

    override fun initialize() {
        hookSolenoid.set(false)
    }

    override fun isFinished() = false

}

