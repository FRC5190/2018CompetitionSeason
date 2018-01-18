/**
 * FRC Team 5190
 * Programming Team
 */


package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.NavX


/**
 * Command that runs autonomous. This class selects the appropriate MP and sends it to the AutoPath class which
 * pushes the information to the Talon.
 * @param path The path to take during autonomous as provided by the Sendable Chooser.
 */
class AutoCommand(private val path: AutoHelper) : Command() {

    init {
        requires(DriveSubsystem)
        NavX.zeroYaw()
    }

    // Instances of AutoPath classes for each side of the DriveTrain
    private lateinit var leftMotionProfile: AutoPath
    private lateinit var rightMotionProfile: AutoPath

    /**
     * Runs once whenever the command is started.
     */
    override fun initialize() {
        leftMotionProfile = AutoPath(DriveSubsystem.falconDrive.leftMaster, path.trajectoryLeft, path.trajectoryLeftDetailed, true, path)
        leftMotionProfile.startMotionProfile()

        rightMotionProfile = AutoPath(DriveSubsystem.falconDrive.rightMaster, path.trajectoryRight, path.trajectoryRightDetailed, false, path)
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