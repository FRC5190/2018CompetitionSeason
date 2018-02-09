package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command

class ResetElevatorCommand(private val debugging: Boolean = false) : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    override fun execute() {
        when (debugging) {
            true -> ElevatorSubsystem.set(ControlMode.PercentOutput, -0.3)
            false -> ElevatorSubsystem.set(ControlMode.PercentOutput, if (ElevatorSubsystem.hasReset) 0.0 else -0.3)
        }
    }


    override fun end() {
        ElevatorSubsystem.hasReset = true
        ElevatorSubsystem.set(ControlMode.PercentOutput, 0.0)
    }

    override fun isFinished() = ElevatorSubsystem.isElevatorAtBottom || (!debugging && ElevatorSubsystem.hasReset)

}