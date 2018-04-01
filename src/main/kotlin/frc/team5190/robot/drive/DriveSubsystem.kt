/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.MainXbox
import frc.team5190.robot.util.*

object DriveSubsystem : Subsystem() {

    // Establishes the drive mode for our different drivers.
    var controlMode = DriveMode.CURVE

    var controller = "Xbox"
    var compressor = Compressor(SolenoidIDs.PCM)

    init {
        compressor.start()
    }

    // Creates an instance of FalconDrive, our custom drive class
    val falconDrive = FalconDrive(listOf(MotorIDs.FRONT_LEFT, MotorIDs.REAR_LEFT).map { WPI_TalonSRX(it) },
            listOf(MotorIDs.FRONT_RIGHT, MotorIDs.REAR_RIGHT).map { WPI_TalonSRX(it) },
            Solenoid(SolenoidIDs.PCM, SolenoidIDs.DRIVE))

    /**
     * Initializes the default command for the subsystem
     */
    override fun initDefaultCommand() {
        this.defaultCommand = ManualDriveCommand()
    }

    /**
     * Executed periodically. Used for switching drive modes.
     */
    override fun periodic() {
        falconDrive.feedSafety()


        SmartDashboard.putNumber("Left Encoder Position", falconDrive.leftEncoderPosition.toDouble())
        SmartDashboard.putNumber("Right Encoder Position", falconDrive.rightEncoderPosition.toDouble())

        if (MainXbox.getStickButtonPressed(GenericHID.Hand.kRight)) {
            controlMode = if (controlMode == DriveMode.CURVE) DriveMode.TANK else DriveMode.CURVE
        }
    }

    /**
     * Resets the DriveTrain in Teleop mode
     */
    fun teleopReset() = falconDrive.teleopReset()

    /**
     * Resets the DriveTrain in Autonomous mode
     */
    fun autoReset() = falconDrive.autoReset()

    fun resetEncoders() = falconDrive.allMasters.forEach {it.setSelectedSensorPosition(0, 0, TIMEOUT)}

}

/**
 * Used for storing the various drive modes.
 */
enum class DriveMode {
    ARCADE,
    TANK,
    CURVE
}
