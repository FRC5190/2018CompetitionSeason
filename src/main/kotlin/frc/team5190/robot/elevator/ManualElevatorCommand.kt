/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.elevator

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.Controls

/**
 * Command that operates elevator based on controller input
 */
class ManualElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    /**
     * Executed periodically
     */
    override fun execute() = Controls.elevatorSubsystem()

    /**
     * Never finishes because it is the default command
     */
    override fun isFinished() = false
}