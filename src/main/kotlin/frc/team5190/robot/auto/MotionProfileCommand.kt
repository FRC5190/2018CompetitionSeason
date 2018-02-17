/**
 * FRC Team 5190
 * Programming Team
 */
package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.pathreader.Pathreader

class MotionProfileCommand(private val requestId: Int, private val isReversed: Boolean = false, private val isMirrored: Boolean = false) : Command() {
    private lateinit var motionProfile: MotionProfile

    init {
        requires(DriveSubsystem)
    }

    /**
     * Runs once whenever the command is started.
     */
    override fun initialize() {
        motionProfile = MotionProfile(
                DriveSubsystem.falconDrive.leftMaster,
                when (isMirrored) {
                    false -> Pathreader.getLeftPath(requestId)!!
                    true -> Pathreader.getRightPath(requestId)!!
                },
                DriveSubsystem.falconDrive.rightMaster,
                when (isMirrored) {
                    false -> Pathreader.getRightPath(requestId)!!
                    true -> Pathreader.getLeftPath(requestId)!!
                },
                isReversed)
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
        DriveSubsystem.falconDrive.leftMotors.forEach { it.inverted = false }
        DriveSubsystem.falconDrive.rightMotors.forEach { it.inverted = true }

        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    /**
     * Ends the command when both sides of the DriveTrain have triggerState executing the motion profile.
     */
    override fun isFinished() = motionProfile.hasFinished()
}