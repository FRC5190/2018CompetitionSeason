/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command

class IntakeHoldCommand : Command() {

    init {
        requires(IntakeSubsystem)
    }

    // Initializes command
    override fun initialize() {
        // Engages intake piston
        IntakeSubsystem.intakeSolenoid.set(true)
        IntakeSubsystem.set(ControlMode.PercentOutput, 0.0)
    }

    // Command never finishes because it is the default command
    override fun isFinished() = false
}