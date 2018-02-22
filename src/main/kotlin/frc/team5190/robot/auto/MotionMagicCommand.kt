/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan S, Prateek M
 */

package frc.team5190.robot.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.util.DriveConstants
import frc.team5190.robot.util.Maths
import kotlin.math.absoluteValue

class MotionMagicCommand(feet: Double,
                         private val cruiseVel: Double = DriveConstants.MOTION_MAGIC_CRUISE,
                         private val accel: Double = DriveConstants.MOTION_MAGIC_ACCEL) : Command() {

    private val setPoint = Maths.feetToNativeUnits(feet, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS).toDouble()

    init {
        requires(DriveSubsystem)
    }

    override fun initialize() {
        DriveSubsystem.falconDrive.allMasters.forEach {
            it.configMotionCruiseVelocity(Maths.feetPerSecondToNativeUnitsPer100Ms(cruiseVel, DriveConstants.WHEEL_RADIUS, DriveConstants.SENSOR_UNITS_PER_ROTATION).toInt(), 10)
            it.configMotionAcceleration(Maths.feetPerSecondToNativeUnitsPer100Ms(accel, DriveConstants.WHEEL_RADIUS, DriveConstants.SENSOR_UNITS_PER_ROTATION).toInt(), 10)

            it.sensorCollection.setQuadraturePosition(0, 10)
            it.set(ControlMode.MotionMagic, setPoint)
        }
    }

    override fun end() {
        DriveSubsystem.falconDrive.leftMotors.forEach { it.inverted = false }
        DriveSubsystem.falconDrive.rightMotors.forEach { it.inverted = true }

        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)
    }

    override fun isFinished() = DriveSubsystem.falconDrive.allMasters.any {
        (it.sensorCollection.quadraturePosition - setPoint).absoluteValue < Maths.feetToNativeUnits(0.1, DriveConstants.SENSOR_UNITS_PER_ROTATION, DriveConstants.WHEEL_RADIUS).toDouble() &&
                it.sensorCollection.quadratureVelocity < 100
    }

}