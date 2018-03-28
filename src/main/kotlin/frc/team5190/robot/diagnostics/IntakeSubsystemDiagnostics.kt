/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.diagnostics

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.intake.IntakeSubsystem

class IntakeSubsystemDiagnostics : TimedCommand(3.0) {

    init {
        requires(IntakeSubsystem)
    }

    override fun execute() {
        IntakeSubsystem.set(ControlMode.PercentOutput, -1.0)
    }

    override fun end() {
        if (IntakeSubsystem.isCubeIn) {
            println("Intake Subsystem OK")
        } else {
            println("Intake Subsystem FAILED")
        }
    }

}