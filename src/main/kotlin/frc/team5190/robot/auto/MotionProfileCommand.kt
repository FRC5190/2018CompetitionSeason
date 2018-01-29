/**
 * FRC Team 5190
 * Programming Team
 */


package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem


/**
 * Command that runs autonomous. This class selects the appropriate MP and sends it to the MotionProfile class which
 * pushes the information to the Talon.
 * @param path The path to take during autonomous as provided by the Sendable Chooser.
 */
class MotionProfileCommand(private val path: Paths, private var isReversed: Boolean = false) : Command() {

    init {
        requires(DriveSubsystem)
    }

    // Instances of MotionProfile classes for the DriveTrain
    private lateinit var motionProfile: MotionProfile

    /**
     * Runs once whenever the command is started.
     */
    override fun initialize() {
        motionProfile = MotionProfile(DriveSubsystem.falconDrive.leftMaster, path.trajectoryLeft, DriveSubsystem.falconDrive.rightMaster, path.trajectoryRight, isReversed)
        motionProfile.startMotionProfile()
    }

    /**
     * Called periodically until triggerState or until the command has been canceled.
     */
    override fun execute() {
        motionProfile.control()

        DriveSubsystem.falconDrive.leftMaster.set(ControlMode.MotionProfile, motionProfile.getSetValue().value.toDouble())
        DriveSubsystem.falconDrive.rightMaster.set(ControlMode.MotionProfile, motionProfile.getSetValue().value.toDouble())

        DriveSubsystem.falconDrive.feedSafety()
    }

    /**
     * Called when the command has ended.
     */
    override fun end() {
        motionProfile.reset()
        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    /**
     * Ends the command when both sides of the DriveTrain have triggerState executing the motion profile.
     */
    override fun isFinished() = motionProfile.hasFinished()
}