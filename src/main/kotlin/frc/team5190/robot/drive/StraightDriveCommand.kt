/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.*
import kotlin.math.absoluteValue


open class StraightDriveCommand(private val distance: Double,
                                private val cruiseVel: Double = DriveConstants.MOTION_MAGIC_CRUISE,
                                private val accel: Double = DriveConstants.MOTION_MAGIC_ACCEL) : Command() {


    // Setpoint in Native Units
    private var setPoint: Double? = null


    init {
        this.requires(DriveSubsystem)
    }

    // Initializes the command
    override fun initialize() {
        // Motion magic setpoint
        setPoint = Maths.feetToNativeUnits(distance, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS).toDouble()

        DriveSubsystem.falconDrive.allMasters.forEach {
            it.configMotionCruiseVelocity(Maths.feetPerSecondToNativeUnitsPer100Ms(cruiseVel, DriveConstants.WHEEL_RADIUS, DriveConstants.SENSOR_UNITS_PER_ROTATION).toInt(), TIMEOUT)
            it.configMotionAcceleration(Maths.feetPerSecondToNativeUnitsPer100Ms(accel, DriveConstants.WHEEL_RADIUS, DriveConstants.SENSOR_UNITS_PER_ROTATION).toInt(), TIMEOUT)

            it.sensorCollection.setQuadraturePosition(0, TIMEOUT)
            it.set(ControlMode.MotionMagic, setPoint!!)
        }
    }

    // Called when the command ends
    override fun end() {
        DriveSubsystem.falconDrive.leftMotors.forEach { it.inverted = false }
        DriveSubsystem.falconDrive.rightMotors.forEach { it.inverted = true }

        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    // Checks command for completion
    override fun isFinished() = DriveSubsystem.falconDrive.allMasters.any {
        (it.sensorCollection.quadraturePosition - setPoint!!).absoluteValue < Maths.feetToNativeUnits(0.1, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS).toDouble() &&
                it.sensorCollection.quadratureVelocity < 100
    }

}