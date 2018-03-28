/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.diagnostics

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.arm.ArmSubsystem

class ArmSubsystemDiagnostics : TimedCommand(3.0) {

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
        ArmSubsystem.set(ControlMode.PercentOutput, 0.05)
    }

    override fun end() {
        if (ArmSubsystem.currentPosition > initialEncoderPosition!! + 150) {
            println("Arm Subsystem OK")
            passedTest = true
        } else {
            println("Arm Subsystem FAILED")
        }
    }

}