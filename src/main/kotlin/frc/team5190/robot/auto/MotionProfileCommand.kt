/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import frc.team5190.robot.drive.DriveSubsystem

/**
 * Command that executes a Motion Profile
 * @param requestId Request ID of the MP
 * @param isReversed If the MP should run in reverse
 * @param isMirrored IF the MP should be mirrored
 */
class MotionProfileCommand(requestId: Int,
                           isReversed: Boolean = false,
                           isMirrored: Boolean = false) : MotionCommand(requestId, isReversed, isMirrored) {

    private lateinit var motionProfile: MotionProfile

    /**
     * Runs once whenever the command is started.
     */
    override fun initialize() {
        motionProfile = MotionProfile(DriveSubsystem.falconDrive.leftMaster, leftTrajectory, DriveSubsystem.falconDrive.rightMaster, rightTrajectory, isReversed)
        motionProfile.startMotionProfile()
    }

    /**
     * Called periodically until triggerState or until the command has been canceled.
     */
    override fun execute() {
        motionProfile.control()
        DriveSubsystem.falconDrive.leftMaster.set(ControlMode.MotionProfile, motionProfile.getSetValue().value.toDouble())
        DriveSubsystem.falconDrive.rightMaster.set(ControlMode.MotionProfile, motionProfile.getSetValue().value.toDouble())
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