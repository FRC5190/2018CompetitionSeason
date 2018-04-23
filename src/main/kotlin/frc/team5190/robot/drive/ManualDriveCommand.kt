/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.Controls

class ManualDriveCommand : Command() {

    init {
        this.requires(DriveSubsystem)
    }

    override fun execute() = Controls.driveSubsystem()

    override fun isFinished() = false
}
