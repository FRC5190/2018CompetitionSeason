/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import frc.team5190.robot.util.*

/**
 * Custom FalconDrive object that extends Differential Drive
 */
class FalconDrive(val leftMotors: List<WPI_TalonSRX>,
                  val rightMotors: List<WPI_TalonSRX>,
                  private val gearSolenoid: Solenoid) : DifferentialDrive(leftMotors[0], rightMotors[0]) {

    // Values for the left side of the DriveTrain
    val leftMaster = leftMotors[0]
    private val leftSlaves = leftMotors.subList(1, leftMotors.size)

    // Values for the right side of the DriveTrain
    val rightMaster = rightMotors[0]
    private val rightSlaves = rightMotors.subList(1, rightMotors.size)

    // Values for all the master motors of the DriveTrain
    val allMasters = listOf(leftMaster, rightMaster)

    // Values for all the motors of the Drive Train
    private val allMotors = listOf(*leftMotors.toTypedArray(), *rightMotors.toTypedArray())


    init {
        reset()
    }

    // Reset
    private fun reset() {
        leftMotors.forEach { it.inverted = false }
        rightMotors.forEach { it.inverted = true }

        leftSlaves.forEach { it.follow(leftMaster) }
        rightSlaves.forEach { it.follow(rightMaster) }

        allMasters.forEach {
            it.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT)
            it.setSelectedSensorPosition(0, 0, TIMEOUT)
            it.configPeakOutput(1.0, -1.0, TIMEOUT)
            it.configMotionProfileTrajectoryPeriod(10, TIMEOUT)
            it.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT)
            it.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT)
        }

        gear = Gear.HIGH

        leftMotors.forEach { it.setSensorPhase(false) }
        rightMotors.forEach { it.setSensorPhase(false) }

        allMotors.forEach {
            it.setNeutralMode(NeutralMode.Brake)
            it.configContinuousCurrentLimit(37, TIMEOUT)
            it.configPeakCurrentDuration(0, TIMEOUT)
            it.configPeakCurrentLimit(0, TIMEOUT)
            it.enableCurrentLimit(true)
        }
    }

    // Auto reset
    internal fun autoReset() {
        this.reset()
    }

    // Teleop reset
    internal fun teleopReset() {
        this.reset()
        allMasters.forEach {
            it.clearMotionProfileTrajectories()
            it.selectProfileSlot(0, 0)
        }
    }

    // Used to set high and low gear of the DriveTrain
    var gear
        get() = Gear.getGear(gearSolenoid.get())
        set(value) {
            when (value) {
                Gear.HIGH -> allMasters.forEach {
                    it.configPIDF(DriveConstants.PID_SLOT_HIGH, DriveConstants.P_HIGH, DriveConstants.I_HIGH, DriveConstants.D_HIGH, DriveConstants.MAX_RPM_HIGH, DriveConstants.SENSOR_UNITS_PER_ROTATION)
                    it.selectProfileSlot(DriveConstants.PID_SLOT_HIGH, 0)
                    it.configOpenloopRamp(0.0, TIMEOUT)
                }
                Gear.LOW -> allMasters.forEach {
                    it.configPIDF(DriveConstants.PID_SLOT_LOW, DriveConstants.P_LOW, DriveConstants.I_LOW, DriveConstants.D_LOW, DriveConstants.MAX_RPM_LOW, DriveConstants.SENSOR_UNITS_PER_ROTATION)
                    it.selectProfileSlot(DriveConstants.PID_SLOT_LOW, 0)
                    it.configOpenloopRamp(0.2, TIMEOUT)
                }
            }
            gearSolenoid.set(value.state)
        }

    // Returns left motor encoder position
    val leftEncoderPosition
        get() = leftMaster.getSelectedSensorPosition(0)

    // Returns right motor encoder position
    val rightEncoderPosition
        get() = rightMaster.getSelectedSensorPosition(0)

    // Feeds motor safety
    fun feedSafety() {
        m_safetyHelper.feed()
    }

    // Controls motors in tank drive motion
    fun tankDrive(controlMode: ControlMode, _leftSpeed: Double, _rightSpeed: Double, squaredInputs: Boolean = true) {
        var leftSpeed = _leftSpeed
        var rightSpeed = _rightSpeed

        leftSpeed = limit(leftSpeed)
        leftSpeed = applyDeadband(leftSpeed, m_deadband)

        rightSpeed = limit(rightSpeed)
        rightSpeed = applyDeadband(rightSpeed, m_deadband)

        // Square the inputs (while preserving the sign) to increase fine control while permitting full power.
        if (squaredInputs) {
            leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed)
            rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed)
        }

        leftMaster.set(controlMode, leftSpeed * controlMode.scale() * m_maxOutput)
        rightMaster.set(controlMode, rightSpeed * controlMode.scale() * m_maxOutput)

        feedSafety()
    }

    // Variables to control Curvature Drive
    private var stopThreshold = kDefaultQuickStopThreshold
    private var stopAlpha = kDefaultQuickStopAlpha
    private var stopAccumulator = 0.0

    // Drives motor in Curve drive motion
    fun curvatureDrive(controlMode: ControlMode, xSpeed: Double, zRotation: Double, isQuickTurn: Boolean) {
        var speed = xSpeed
        var rotation = zRotation

        speed = limit(speed)
        speed = applyDeadband(speed, m_deadband)

        rotation = limit(rotation)
        rotation = applyDeadband(rotation, m_deadband)

        val angularPower: Double
        val overPower: Boolean

        if (isQuickTurn) {
            if (Math.abs(speed) < stopThreshold) {
                stopAccumulator = (1 - stopAlpha) * stopAccumulator + stopAlpha * limit(rotation) * 2.0
            }
            overPower = true
            angularPower = rotation
        } else {
            overPower = false
            angularPower = Math.abs(speed) * rotation - stopAccumulator

            when {
                stopAccumulator > 1 -> stopAccumulator -= 1.0
                stopAccumulator < -1 -> stopAccumulator += 1.0
                else -> stopAccumulator = 0.0
            }
        }

        var leftMotorOutput = speed + angularPower
        var rightMotorOutput = speed - angularPower

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            when {
                leftMotorOutput > 1.0 -> {
                    rightMotorOutput -= leftMotorOutput - 1.0
                    leftMotorOutput = 1.0
                }
                rightMotorOutput > 1.0 -> {
                    leftMotorOutput -= rightMotorOutput - 1.0
                    rightMotorOutput = 1.0
                }
                leftMotorOutput < -1.0 -> {
                    rightMotorOutput -= leftMotorOutput + 1.0
                    leftMotorOutput = -1.0
                }
                rightMotorOutput < -1.0 -> {
                    leftMotorOutput -= rightMotorOutput + 1.0
                    rightMotorOutput = -1.0
                }
            }
        }

        leftMaster.set(controlMode, leftMotorOutput * controlMode.scale() * m_maxOutput)
        rightMaster.set(controlMode, rightMotorOutput * controlMode.scale() * m_maxOutput)

        feedSafety()
    }
}

// Enum that contains the gear states
enum class Gear(val state: Boolean) {
    HIGH(false), LOW(true);

    companion object {
        fun getGear(solenoidState: Boolean) = Gear.values().find { it.state == solenoidState }!!
    }
}