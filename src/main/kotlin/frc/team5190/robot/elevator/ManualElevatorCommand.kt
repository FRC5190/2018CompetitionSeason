/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox
import frc.team5190.robot.getTriggerPressed
import frc.team5190.robot.util.ElevatorConstants

/**
 * Command that operates elevator based on controller input
 */
class ManualElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    // Latch boolean
    var triggerState = false

    /**
     * Executed periodically
     */
    override fun execute() {
        when {
            MainXbox.getTriggerPressed(GenericHID.Hand.kRight) -> {
                val motorOut =  0.5
                ElevatorSubsystem.setPeakOutput(ElevatorConstants.ACTIVE_PEAK_OUT)
                ElevatorSubsystem.set(ControlMode.PercentOutput, motorOut)
                triggerState = true
            }
            triggerState -> {
                ElevatorSubsystem.setPeakOutput(ElevatorConstants.IDLE_PEAK_OUT)
                ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition + 500.0)
                triggerState = false
            }
        }
        when {
            MainXbox.getBumper(GenericHID.Hand.kRight) -> {
                ElevatorSubsystem.setPeakOutput(ElevatorConstants.ACTIVE_PEAK_OUT)
                val motorOut = -0.1
                ElevatorSubsystem.set(ControlMode.PercentOutput, motorOut)
            }
            MainXbox.getBumperReleased(GenericHID.Hand.kRight) -> {
                ElevatorSubsystem.setPeakOutput(ElevatorConstants.IDLE_PEAK_OUT)
                ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition - 500.0)
            }
        }
    }

    /**
     * Never finishes because it is the default command
     */
    override fun isFinished() = false
}