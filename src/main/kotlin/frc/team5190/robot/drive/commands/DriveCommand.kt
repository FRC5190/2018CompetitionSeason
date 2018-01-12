package frc.team5190.robot.drive.commands

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem

abstract class DriveCommand : Command() {

    init {
        this.requires(DriveSubsystem)
    }

}
