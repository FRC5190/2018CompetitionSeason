package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox

class ManualElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    var triggerState = false

    override fun execute() {
        when {
            MainXbox.getTriggerAxis(GenericHID.Hand.kRight) > 0.5 -> {
                ElevatorSubsystem.set(ControlMode.PercentOutput, -0.1)
                triggerState = true
            }
            MainXbox.getBumper(GenericHID.Hand.kRight) -> {
                ElevatorSubsystem.set(ControlMode.PercentOutput, 0.4)
            }
            MainXbox.getBumperReleased(GenericHID.Hand.kRight) -> ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition + 500)

            MainXbox.getTriggerAxis(GenericHID.Hand.kRight) < 0.5 && triggerState -> {
                ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition - 1440)
                triggerState = false
            }
        }
    }

    override fun isFinished() = false
}