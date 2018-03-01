package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.pathreader.Pathreader

abstract class MotionCommand(protected val requestId: Int,
                             protected val isReversed: Boolean = false,
                             protected val isMirrored: Boolean = false) : Command() {

    init {
        this.requires(DriveSubsystem)
    }

    protected val leftTrajectory by lazy {
        when (isMirrored) {
            false -> Pathreader.getLeftPath(requestId)!!
            true -> Pathreader.getRightPath(requestId)!!
        }
    }
    protected val rightTrajectory by lazy {
        when (isMirrored) {
            false -> Pathreader.getRightPath(requestId)!!
            true -> Pathreader.getLeftPath(requestId)!!
        }
    }

    fun getMPTime(): Double {
        return if (rightTrajectory.map { it.dt }.sum() == leftTrajectory.map { it.dt }.sum())
            leftTrajectory.map { it.dt }.sum()
        else 0.0
    }

}