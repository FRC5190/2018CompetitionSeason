package frc.team5190.robot.drive.commands

import frc.team5190.robot.drive.DriveSubsystem

class TeleDriveCommand : DriveCommand() {

    override fun execute() {
        DriveSubsystem.drive()
    }

    override fun isFinished() = false
}
