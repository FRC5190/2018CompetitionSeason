package frc.team5190.robot

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveTrain

class Reset() : Command() {

    init {
        requires(DriveTrain)
    }

    override fun execute() = DriveTrain.reset()

    override fun isFinished(): Boolean  = true
}