/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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

    // Default command
    override fun initDefaultCommand() {
        this.defaultCommand = ManualDriveCommand()
    }

    // Periodic 50hz loop
    override fun periodic() {
        falconDrive.feedSafety()

        SmartDashboard.putNumber("Left Encoder Position", falconDrive.leftEncoderPosition.toDouble())
        SmartDashboard.putNumber("Right Encoder Position", falconDrive.rightEncoderPosition.toDouble())

        SmartDashboard.putNumber("Left Power", falconDrive.leftMaster.outputCurrent)
        SmartDashboard.putNumber("Right Power", falconDrive.rightMaster.outputCurrent)

    }

    // Teleop drive reset
    fun teleopReset() = falconDrive.teleopReset()

    // Auto drive reset
    fun autoReset() = falconDrive.autoReset()

    // Encoder reset
    fun resetEncoders() = falconDrive.allMasters.forEach {it.setSelectedSensorPosition(0, 0, TIMEOUT)}

}

// Drive modes
enum class DriveMode {
    ARCADE,
    TANK,
    CURVE
}
