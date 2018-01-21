package frc.team5190.robot.elevator

import edu.wpi.first.wpilibj.command.Command

class ResetElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    override fun execute() {
        ElevatorSubsystem.resetEncoders()
    }

    override fun isFinished(): Boolean {
        return true
    }

}