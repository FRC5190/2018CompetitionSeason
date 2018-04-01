/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.diagnostics

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.arm.ArmSubsystem

class ArmSubsystemDiagnostics : TimedCommand(1.5) {

    val hasPassedTest
        get() = passedTest

    private var initialEncoderPosition: Int? = null
    private var passedTest = false

    init {
        requires(ArmSubsystem)
    }

    override fun initialize() {
        initialEncoderPosition = ArmSubsystem.currentPosition
    }

    override fun execute() {
        ArmSubsystem.set(ControlMode.PercentOutput, 0.2)
    }

    override fun end() {
        ArmSubsystem.set(ControlMode.PercentOutput, 0.0)
        if (ArmSubsystem.currentPosition > initialEncoderPosition!! + 50) {
            println("Arm Subsystem OK")
            passedTest = true
        } else {
            println("Arm Subsystem FAILED")
        }
    }

}