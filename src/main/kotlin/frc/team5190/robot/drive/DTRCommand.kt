package frc.team5190.robot.drive

import edu.wpi.first.wpilibj.command.Command

class DriveCommand : Command() {

    init {
        requires(DriveTrain)
    }

    override fun execute() {
        DriveTrain.driveWithXbox()
    }

    override fun isFinished() = false
}