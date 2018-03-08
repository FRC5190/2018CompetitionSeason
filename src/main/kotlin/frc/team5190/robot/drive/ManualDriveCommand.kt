/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.Controls

class ManualDriveCommand : Command() {

    init {
        this.requires(DriveSubsystem)
    }

    /**
     * Called periodically until the command is triggerState or until interrupted.
     */
    override fun execute() = Controls.driveSubsystem()

    override fun isFinished() = false
}
