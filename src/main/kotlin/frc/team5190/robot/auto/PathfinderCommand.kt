package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.Notifier
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.pathreader.MotionProfileTrajectory
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.DriveConstants
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory
import jaci.pathfinder.followers.EncoderFollower

/**
 * Command that executes a Motion Profile using Jaci's Pathfinder
 * @param requestId Request ID of the MP
 * @param isReversed If the MP should run in reverse
 * @param isMirrored IF the MP should be mirrored
 */
class PathfinderCommand(requestId: Int,
                        isReversed: Boolean = false,
                        isMirrored: Boolean = false) : MotionCommand(requestId, isReversed, isMirrored) {

    private val profileThread = Notifier({
        val encoderPosition = with(DriveSubsystem.falconDrive) { leftEncoderPosition + rightEncoderPosition } / 2

        val leftOutput = leftFollower.calculate(encoderPosition)
        val rightOutput = rightFollower.calculate(encoderPosition)

        val gyroHeading = NavX.angle
        val desiredHeading = Pathfinder.r2d((Pathfinder.r2d(leftFollower.heading) + Pathfinder.r2d(rightFollower.heading)) / 2.0)

        val angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading)
        val turn = 0.8 * (-1.0 / 80.0) * angleDifference

        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, leftOutput + turn, rightOutput - turn)
    })

    private val leftFollower by lazy { convertToFollower(leftTrajectory) }

    private val rightFollower by lazy { convertToFollower(rightTrajectory) }

    private fun convertToFollower(motionProfileTrajectory: MotionProfileTrajectory) = EncoderFollower(Trajectory(motionProfileTrajectory.map {
        Trajectory.Segment(it.dt, it.x, it.y, it.position, it.velocity, it.acceleration, it.jerk, it.heading)
    }.toTypedArray())).apply {
        configureEncoder(0, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS / 6.0)
        configurePIDVA(DriveConstants.P_HIGH, DriveConstants.I_HIGH, DriveConstants.D_HIGH, 1.0 / 9.0, 0.0)
    }

    override fun initialize() {
        DriveSubsystem.autoReset()
        profileThread.startPeriodic(leftTrajectory[0].dt)
    }

    override fun end() {
        profileThread.stop()
    }

    override fun isFinished() = leftFollower.isFinished || rightFollower.isFinished

}