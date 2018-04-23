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

    override fun initialize() {
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.ACTIVE_PEAK_OUT
        ElevatorSubsystem.set(ControlMode.MotionMagic, ticks)
    }

    override fun end() {
        ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT
    }

    override fun isFinished() = (ElevatorSubsystem.currentPosition - ticks).absoluteValue < ElevatorSubsystem.inchesToNativeUnits(1.0)
}
