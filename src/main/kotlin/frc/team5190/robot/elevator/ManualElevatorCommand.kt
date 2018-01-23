package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox

class ManualElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    override fun execute() {
        when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) -> ElevatorSubsystem.set(0.3)
            MainXbox.getBumper(GenericHID.Hand.kRight) -> ElevatorSubsystem.set(-0.3)
            MainXbox.getBumperReleased(GenericHID.Hand.kLeft) ->
                ElevatorSubsystem.set(ControlMode.Position, ElevatorSubsystem.position + 500)
            MainXbox.getBumperReleased(GenericHID.Hand.kRight) ->
                ElevatorSubsystem.set(ControlMode.Position, ElevatorSubsystem.position - 1440)
        }
    }

    override fun isFinished() = false
}