/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan S, Prateek M
 */

/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.util.MotorIDs
import frc.team5190.robot.util.SolenoidIDs

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
    }

    fun teleopReset() = falconDrive.teleopReset()

    fun autoReset() = falconDrive.autoReset()

}

/**
 * Used for storing the various drive modes.
 */
enum class DriveMode {
    ARCADE,
    TANK,
    CURVE
}
