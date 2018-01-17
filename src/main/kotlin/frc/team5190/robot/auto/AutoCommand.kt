/**
 * FRC Team 5190
 * Programming Team
 */


package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem


/**
 * Command that runs autonomous
 */
class AutoCommand(private val path: AutoHelper) : Command() {

    init {
        requires(DriveSubsystem)
    }

    // Instances of AutoPath classes for each side of the DriveTrain
    private lateinit var leftMotionProfile: AutoPath
    private lateinit var rightMotionProfile: AutoPath

    /**
     * Runs once whenever the command is started.
     */
    override fun initialize() {
        leftMotionProfile = AutoPath(DriveSubsystem.falconDrive.leftMaster, path.trajectoryLeft)
        leftMotionProfile.startMotionProfile()

        rightMotionProfile = AutoPath(DriveSubsystem.falconDrive.rightMaster, path.trajectoryRight)
        rightMotionProfile.startMotionProfile()
    }

    /**
     * Called periodically until finished or until the command has been canceled.
     */
    override fun execute() {
        leftMotionProfile.control()
        rightMotionProfile.control()

        DriveSubsystem.falconDrive.leftMaster.set(ControlMode.MotionProfile, leftMotionProfile.getSetValue().value.toDouble())
        DriveSubsystem.falconDrive.rightMaster.set(ControlMode.MotionProfile, rightMotionProfile.getSetValue().value.toDouble())

        DriveSubsystem.falconDrive.feedSafety()
    }

    /**
     * Called when the command has ended.
     */
    override fun end() {
        leftMotionProfile.reset()
        rightMotionProfile.reset()
        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    /**
     * Ends the command when both sides of the DriveTrain have finished executing the motion profile.
     */
    override fun isFinished() = leftMotionProfile.hasFinished() && rightMotionProfile.hasFinished()
}