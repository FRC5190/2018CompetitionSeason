package frc.team5190.robot.elevator

import edu.wpi.first.wpilibj.command.Command

class ResetElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    override fun execute() {
        ElevatorSubsystem.resetEncoders()
        //ElevatorSubsystem.set(-0.3)
    }

    /*
    override fun end() {
        ElevatorSubsystem.set(0.0)
    }

    override fun isFinished() = ElevatorSubsystem.elevatorAtBottom
*/
    override fun isFinished(): Boolean = false

}