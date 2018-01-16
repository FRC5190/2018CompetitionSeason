package frc.team5190.robot.navigation

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveTrain

class TRNCommand : Command() {

    init {
        requires(DriveTrain)
    }

    private val turnController = TRNController(-90.0)

    override fun initialize() {
        turnController.enable()
    }

    override fun execute() {
        turnController.rotateToAngle()
    }

    override fun isFinished() = turnController.hasFinished()

}