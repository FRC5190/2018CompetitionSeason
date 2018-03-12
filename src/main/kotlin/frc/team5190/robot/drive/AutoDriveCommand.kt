/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.*
import kotlin.math.absoluteValue

/**
 * Command that drives to distance
 * @param feet Distance to go forward
 * @param cruiseVel Cruise velocity
 * @param accel Acceleration
 */
open class AutoDriveCommand(feet: Double,
                       private val cruiseVel: Double = DriveConstants.MOTION_MAGIC_CRUISE,
                       private val accel: Double = DriveConstants.MOTION_MAGIC_ACCEL) : Command() {

    // Setpoint in Native Units
    private val setPoint = Maths.feetToNativeUnits(feet, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS).toDouble()

    init {
        this.requires(DriveSubsystem)
    }

    /**
     * Initializes the command
     */
    override fun initialize() {
        DriveSubsystem.falconDrive.allMasters.forEach {
            it.configMotionCruiseVelocity(Maths.feetPerSecondToNativeUnitsPer100Ms(cruiseVel, DriveConstants.WHEEL_RADIUS, DriveConstants.SENSOR_UNITS_PER_ROTATION).toInt(), TIMEOUT)
            it.configMotionAcceleration(Maths.feetPerSecondToNativeUnitsPer100Ms(accel, DriveConstants.WHEEL_RADIUS, DriveConstants.SENSOR_UNITS_PER_ROTATION).toInt(), TIMEOUT)

            it.sensorCollection.setQuadraturePosition(0, TIMEOUT)
            it.set(ControlMode.MotionMagic, setPoint)
        }
    }

    /**
     * Ends the command
     */
    override fun end() {
        DriveSubsystem.falconDrive.leftMotors.forEach { it.inverted = false }
        DriveSubsystem.falconDrive.rightMotors.forEach { it.inverted = true }

        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    /**
     * Checks if the DriveTrain has reached the setpoint
     */
    override fun isFinished() = DriveSubsystem.falconDrive.allMasters.any {
        (it.sensorCollection.quadraturePosition - setPoint).absoluteValue < Maths.feetToNativeUnits(0.1, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS).toDouble() &&
                it.sensorCollection.quadratureVelocity < 100
    }

}