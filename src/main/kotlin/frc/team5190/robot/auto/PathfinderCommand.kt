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

    private val leftFollower by lazy { convertToFollower(if (isReversed) rightTrajectory else leftTrajectory) }

    private val rightFollower by lazy { convertToFollower(if (isReversed) leftTrajectory else rightTrajectory) }

    private fun convertToFollower(motionProfileTrajectory: MotionProfileTrajectory) = EncoderFollower(Trajectory(motionProfileTrajectory.map {
        Trajectory.Segment(it.dt, it.x, it.y, it.position, it.velocity, it.acceleration, it.jerk, it.heading)
    }.toTypedArray())).apply {
        configureEncoder(0, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS * 2.0 / 12.0)
        configurePIDVA(DriveConstants.P_HIGH, DriveConstants.I_HIGH, DriveConstants.D_HIGH, 1.0 / DriveConstants.MAX_STU_HIGH * 100.0, 0.0)
    }

    override fun initialize() {
        DriveSubsystem.autoReset()

        DriveSubsystem.falconDrive.leftMotors.forEach {
            it.inverted = isReversed
            it.setSensorPhase(!DriveConstants.IS_RACE_ROBOT)
        }
        DriveSubsystem.falconDrive.rightMotors.forEach {
            it.inverted = !isReversed
            it.setSensorPhase(!DriveConstants.IS_RACE_ROBOT)
        }

        profileThread.startPeriodic(leftTrajectory[0].dt)
    }

    override fun execute() {
        DriveSubsystem.falconDrive.feedSafety()
    }

    override fun end() {
        profileThread.stop()
        DriveSubsystem.falconDrive.leftMotors.forEach { it.inverted = false }
        DriveSubsystem.falconDrive.rightMotors.forEach { it.inverted = true }

        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    override fun isFinished() = leftFollower.isFinished || rightFollower.isFinished

}