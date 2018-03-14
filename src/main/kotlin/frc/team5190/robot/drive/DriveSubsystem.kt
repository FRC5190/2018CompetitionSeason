/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.Robot
import frc.team5190.robot.util.DriveConstants
import frc.team5190.robot.util.Maths
import frc.team5190.robot.util.MotorIDs
import frc.team5190.robot.util.SolenoidIDs
import kotlin.math.absoluteValue

object DriveSubsystem : Subsystem() {

    // Establishes the drive mode for our different drivers.
    var controlMode = DriveMode.CURVE

    var controller = "Xbox"
    var compressor = Compressor(SolenoidIDs.PCM)

    var autoShift = true

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

        // Auto Shift Logic
        if (Robot.INSTANCE!!.isOperatorControl && autoShift) {
            val speed = falconDrive.allMasters.map { Maths.nativeUnitsPer100MsToFeetPerSecond(it.getSelectedSensorVelocity(0).absoluteValue) }.average()
            when {
                speed > DriveConstants.AUTO_SHIFT_HIGH_THRESHOLD -> falconDrive.gear = Gear.HIGH
                speed < DriveConstants.AUTO_SHIFT_LOW_THRESHOLD -> falconDrive.gear = Gear.LOW
            }
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

}

/**
 * Used for storing the various drive modes.
 */
enum class DriveMode {
    ARCADE,
    TANK,
    CURVE
}
