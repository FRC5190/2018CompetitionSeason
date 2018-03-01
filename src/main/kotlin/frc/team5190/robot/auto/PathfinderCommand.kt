package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.PIDController
import edu.wpi.first.wpilibj.PIDOutput
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

    private var turnControlOutput = 0.0

    private val turnController = PIDController(DriveConstants.TURN_P, DriveConstants.TURN_I, DriveConstants.TURN_D, NavX, PIDOutput { turnControlOutput = it })

    private val leftFollower by lazy {
        EncoderFollower(convertToPathfinder(leftTrajectory))
    }

    private val rightFollower by lazy {
        EncoderFollower(convertToPathfinder(rightTrajectory))
    }

    /**
     * @param The motion project trajectory list that is going to be converted to Jaci's Trajectory list
     * @return The converted trajectory list
     */
    private fun convertToPathfinder(motionProfileTrajectory: MotionProfileTrajectory) =
            Trajectory(motionProfileTrajectory.map {
                Trajectory.Segment(it.dt, it.x, it.y, it.position, it.velocity, it.acceleration, it.jerk, it.heading)
            }.toTypedArray())

    init {
        with(turnController) {
            setInputRange(-180.0, 180.0)
            setOutputRange(-1.0, 1.0)
            setAbsoluteTolerance(5.0)
            setContinuous(true)
        }
    }

    override fun execute() {
        val encoderPosition = with(DriveSubsystem.falconDrive) { leftEncoderPosition + rightEncoderPosition } / 2

        val leftOutput = leftFollower.calculate(encoderPosition)
        val rightOutput = rightFollower.calculate(encoderPosition)
        turnController.setpoint = Pathfinder.boundHalfDegrees((Pathfinder.r2d(leftFollower.heading) + Pathfinder.r2d(rightFollower.heading)) / 2.0)    // Bound to -180..180 degrees

        DriveSubsystem.falconDrive.tankDrive(ControlMode.Velocity, leftOutput + turnControlOutput, rightOutput - turnControlOutput)
    }

    override fun isFinished() = leftFollower.isFinished || rightFollower.isFinished

}