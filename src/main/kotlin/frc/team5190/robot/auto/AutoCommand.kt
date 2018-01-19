/**
 * FRC Team 5190
 * Programming Team
 */


package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox
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
    private lateinit var lMotionProfile: AutoPath
    private lateinit var rMotionProfile: AutoPath

    private var hasBeenPaused = false

    /**
     * Runs once whenever the command is started.
     */
    override fun initialize() {
        lMotionProfile = AutoPath(DriveSubsystem.falconDrive.leftMaster, path.trajectoryLeft, path, true)
        lMotionProfile.startMotionProfile()

        rMotionProfile = AutoPath(DriveSubsystem.falconDrive.rightMaster, path.trajectoryRight, path, false)
        rMotionProfile.startMotionProfile()

        NavX.reset()
    }

    /**
     * Called periodically until finished or until the command has been canceled.
     */
    override fun execute() {

        if (MainXbox.bButton) {
            DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0, false)
            if (!hasBeenPaused) {
//                DriveSubsystem.falconDrive.feedSafety()
                lMotionProfile.pauseMotionProfile()
                rMotionProfile.pauseMotionProfile()
                hasBeenPaused = true
            }
        } else if (hasBeenPaused) {
            lMotionProfile.resumeMotionProfile()
            rMotionProfile.resumeMotionProfile()
            hasBeenPaused = false
        }


        lMotionProfile.control()
        rMotionProfile.control()

        DriveSubsystem.falconDrive.leftMaster.set(ControlMode.MotionProfile, lMotionProfile.getSetValue().value.toDouble())
        DriveSubsystem.falconDrive.rightMaster.set(ControlMode.MotionProfile, rMotionProfile.getSetValue().value.toDouble())

        DriveSubsystem.falconDrive.feedSafety()
    }

    /**
     * Called when the command has ended.
     */
    override fun end() {
        lMotionProfile.reset()
        rMotionProfile.reset()
        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    /**
     * Ends the command when both sides of the DriveTrain have finished executing the motion profile.
     */
    override fun isFinished() = lMotionProfile.hasFinished() && rMotionProfile.hasFinished()
}