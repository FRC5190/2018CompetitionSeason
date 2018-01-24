package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command

class ResetElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    override fun execute() {
        ElevatorSubsystem.set(ControlMode.PercentOutput, -0.3)
    }


    override fun end() {
        ElevatorSubsystem.resetEncoders()
        ElevatorSubsystem.set(ControlMode.PercentOutput, 0.0)
    }

    override fun isFinished() = ElevatorSubsystem.isElevatorAtBottom()


}