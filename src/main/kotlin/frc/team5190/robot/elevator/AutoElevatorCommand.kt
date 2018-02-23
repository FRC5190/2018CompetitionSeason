/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.ElevatorConstants
import kotlin.math.absoluteValue

/**
 * Command that moves the elevator to different presets
 * @param position The position in Native Units that the elevator must travel to
 */
class AutoElevatorCommand(val position: ElevatorPosition) : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    /**
     * Initializes the command
     */
    override fun initialize() {
        ElevatorSubsystem.setPeakOutput(ElevatorConstants.ACTIVE_PEAK_OUT)
        ElevatorSubsystem.set(ControlMode.MotionMagic, position.ticks.toDouble())
    }

    /**
     * Executed when the command ends.
     */
    override fun end() {
        ElevatorSubsystem.setPeakOutput(ElevatorConstants.IDLE_PEAK_OUT)
    }

    /**
     * Checks if the elevator has reached the setpoint
     */
    override fun isFinished() = (ElevatorSubsystem.currentPosition - position.ticks).absoluteValue < ElevatorSubsystem.inchesToNativeUnits(1.0)
}
