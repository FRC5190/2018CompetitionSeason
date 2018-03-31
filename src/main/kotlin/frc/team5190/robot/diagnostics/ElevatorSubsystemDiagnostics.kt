/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.diagnostics

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.elevator.ElevatorSubsystem

class ElevatorSubsystemDiagnostics : TimedCommand(3.0) {

    val hasPassedTest
        get() = passedTest

    private var initialEncoderPosition: Int? = null
    private var passedTest = false

    init {
        requires(ElevatorSubsystem)
    }

    override fun initialize() {
        initialEncoderPosition = ElevatorSubsystem.currentPosition
    }

    override fun execute() {
        ElevatorSubsystem.set(ControlMode.PercentOutput, 0.4)
    }

    override fun end() {
        if (ElevatorSubsystem.currentPosition > initialEncoderPosition!! + 1440) {
            println("Elevator Subsystem OK")
            passedTest = true
        } else {
            println("Elevator Subsystem FAILED")
        }
    }

}