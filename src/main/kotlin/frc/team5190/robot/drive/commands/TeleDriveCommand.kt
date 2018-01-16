package frc.team5190.robot.drive.commands

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox
import frc.team5190.robot.drive.DriveMode
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.getLeftX
import frc.team5190.robot.getLeftY
import frc.team5190.robot.getRightY

class TeleDriveCommand : Command() {

    init {
        this.requires(DriveSubsystem)
    }

    override fun execute() {
        when (DriveSubsystem.controlMode) {
            DriveMode.ARCADE -> DriveSubsystem.falconDrive.arcadeDrive(-MainXbox.getLeftY(), MainXbox.getLeftX())
            DriveMode.TANK -> DriveSubsystem.falconDrive.tankDrive(-MainXbox.getLeftY(), -MainXbox.getRightY())
            DriveMode.CURVE -> DriveSubsystem.falconDrive.curvatureDrive(MainXbox.getLeftY(), MainXbox.getLeftX(), MainXbox.aButton)
        }
    }

    override fun isFinished() = false
}
