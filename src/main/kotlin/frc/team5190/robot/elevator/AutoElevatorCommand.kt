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
open class AutoElevatorCommand(val ticks: Double) : Command() {

    constructor(position: ElevatorPosition) : this(position.ticks.toDouble())


    init {
        this.requires(ElevatorSubsystem)
    }

    /**
     * Initializes the command
     */
    override fun initialize() {
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.ACTIVE_PEAK_OUT
        ElevatorSubsystem.set(ControlMode.MotionMagic, ticks)
    }

    /**
     * Executed when the command ends.
     */
    override fun end() {
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT
    }

    /**
     * Checks if the elevator has reached the setpoint
     */
    override fun isFinished() = (ElevatorSubsystem.currentPosition - ticks).absoluteValue < ElevatorSubsystem.inchesToNativeUnits(1.0)
}
