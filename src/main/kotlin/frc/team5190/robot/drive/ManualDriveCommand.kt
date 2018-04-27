/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.Controls

class ManualDriveCommand : Command() {

    init {
        requires(DriveSubsystem)
    }

    // Executed periodically
    override fun execute() = Controls.driveSubsystem()

    // Command never finishes because it is the default command
    override fun isFinished() = false
}
