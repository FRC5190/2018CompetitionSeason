package frc.team5190.robot.drive.commands

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem

class TeleDriveCommand : Command() {

    init {
        requires(DriveSubsystem)
    }

    override fun execute() {
        DriveSubsystem.drive()
    }

    override fun isFinished() = false
}
