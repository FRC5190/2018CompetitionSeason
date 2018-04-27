/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.ElevatorConstants
import kotlin.math.absoluteValue

open class AutoElevatorCommand(val ticks: Double) : Command() {

    constructor(position: ElevatorPosition) : this(position.ticks.toDouble())

    init {
        this.requires(ElevatorSubsystem)
    }

    // Initializes command
    override fun initialize() {
        // Peak out changed to max and elevator setpoint
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.ACTIVE_PEAK_OUT
        ElevatorSubsystem.set(ControlMode.MotionMagic, ticks)
    }

    // Called when command ends
    override fun end() {
        // Peak out changed back to 20% to not destroy the motors in case encoder fails
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT
    }

    // Checks command for completion
    override fun isFinished() = (ElevatorSubsystem.currentPosition - ticks).absoluteValue < ElevatorSubsystem.inchesToNativeUnits(1.0)
}
