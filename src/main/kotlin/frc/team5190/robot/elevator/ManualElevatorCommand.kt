/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan S, Prateek M
 */

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
                val motorOut =  0.5
                ElevatorSubsystem.set(ControlMode.PercentOutput, motorOut)
                triggerState = true
            }
            triggerState -> {

                ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition + 500.0)
                triggerState = false
            }
        }
        when {
            MainXbox.getBumper(GenericHID.Hand.kRight) -> {
                val motorOut = -0.1
                ElevatorSubsystem.set(ControlMode.PercentOutput, motorOut)
            }
            MainXbox.getBumperReleased(GenericHID.Hand.kRight) -> ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition - 500.0)
        }
    }

    override fun isFinished() = false
}