package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox
import frc.team5190.robot.getTriggerPressed

class ManualElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    var triggerState = false

    override fun execute() {
        when {
            MainXbox.getTriggerPressed(GenericHID.Hand.kRight) -> {
                ElevatorSubsystem.set(ControlMode.PercentOutput, 0.5)
                triggerState = true
            }
            triggerState -> {
                ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition + 500)
                triggerState = false
            }
        }
        when {
            MainXbox.getBumper(GenericHID.Hand.kRight) -> {
                ElevatorSubsystem.set(ControlMode.PercentOutput, -0.1)
            }
            MainXbox.getBumperReleased(GenericHID.Hand.kRight) -> ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition - 1440)
        }
    }

    override fun isFinished() = false
}