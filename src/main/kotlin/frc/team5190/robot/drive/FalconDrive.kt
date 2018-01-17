/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import frc.team5190.robot.util.Hardware
import frc.team5190.robot.util.Maths
import frc.team5190.robot.util.scale

/**
 * Custom FalconDrive object that extends Differential Drive
 */
class FalconDrive(val leftMotors: List<WPI_TalonSRX>,
                  val rightMotors: List<WPI_TalonSRX>) : DifferentialDrive(leftMotors[0], rightMotors[0]) {

    val leftMaster = leftMotors[0]
    private val leftSlaves = leftMotors.subList(1, leftMotors.size)

    val rightMaster = rightMotors[0]
    private val rightSlaves = rightMotors.subList(1, rightMotors.size)

    private val allMasters = listOf(leftMaster, rightMaster)

    private val allMotors = listOf(*leftMotors.toTypedArray(), *rightMotors.toTypedArray())

    init {
        leftSlaves.forEach { it.follow(leftMaster) }
        rightSlaves.forEach { it.follow(rightMaster) }

        allMotors.forEach { it.setNeutralMode(NeutralMode.Brake) }

        configure()
    }

    /**
     * Configures PID values
     */
    fun configure() {
        allMasters.forEach {
            it.configurePIDF(2.0, 0.0, 0.0, 1.0, rpm = Hardware.MAX_RPM.toDouble(),
                    sensorUnitsPerRotation = Hardware.NATIVE_UNITS_PER_ROTATION.toDouble(), dev = FeedbackDevice.QuadEncoder)
            it.configMotionProfileTrajectoryPeriod(10, 10)
            it.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10)
            it.configNeutralDeadband(0.04, 10)
        }

        rightMotors.forEach { it.inverted = true }

        allMotors.forEach {
            it.setSensorPhase(true)
            it.setNeutralMode(NeutralMode.Brake)
        }
    }

    val leftEncoderPosition
        get() = leftMaster.getSelectedSensorPosition(0)

    val rightEncoderPosition
        get() = rightMaster.getSelectedSensorPosition(0)

    fun feedSafety() {
        m_safetyHelper.feed()
    }

    fun tankDrive(controlMode: ControlMode, _leftSpeed: Double, _rightSpeed: Double, squaredInputs: Boolean = true) {
        var leftSpeed = _leftSpeed
        var rightSpeed = _rightSpeed

        leftSpeed = limit(leftSpeed)
        leftSpeed = applyDeadband(leftSpeed, m_deadband)

        rightSpeed = limit(rightSpeed)
        rightSpeed = applyDeadband(rightSpeed, m_deadband)

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squaredInputs) {
            leftSpeed = Math.copySign(leftSpeed * leftSpeed, leftSpeed)
            rightSpeed = Math.copySign(rightSpeed * rightSpeed, rightSpeed)
        }

        leftMaster.set(controlMode, leftSpeed * controlMode.scale() * m_maxOutput)
        rightMaster.set(controlMode, -rightSpeed * controlMode.scale() * m_maxOutput)

        m_safetyHelper.feed()
    }


    private var m_quickStopThreshold = kDefaultQuickStopThreshold
    private var m_quickStopAlpha = kDefaultQuickStopAlpha
    private var m_quickStopAccumulator = 0.0

    fun curvatureDrive(controlMode: ControlMode, xSpeed: Double, zRotation: Double, isQuickTurn: Boolean) {
        var xSpeed = xSpeed
        var zRotation = zRotation

        xSpeed = limit(xSpeed)
        xSpeed = applyDeadband(xSpeed, m_deadband)

        zRotation = limit(zRotation)
        zRotation = applyDeadband(zRotation, m_deadband)

        val angularPower: Double
        val overPower: Boolean

        if (isQuickTurn) {
            if (Math.abs(xSpeed) < m_quickStopThreshold) {
                m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator + m_quickStopAlpha * limit(zRotation) * 2.0
            }
            overPower = true
            angularPower = zRotation
        } else {
            overPower = false
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1.0
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1.0
            } else {
                m_quickStopAccumulator = 0.0
            }
        }

        var leftMotorOutput = xSpeed + angularPower
        var rightMotorOutput = xSpeed - angularPower

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0
                leftMotorOutput = 1.0
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0
                rightMotorOutput = 1.0
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0
                leftMotorOutput = -1.0
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0
                rightMotorOutput = -1.0
            }
        }

        leftMaster.set(controlMode, leftMotorOutput * controlMode.scale() * m_maxOutput)
        rightMaster.set(controlMode, rightMotorOutput * controlMode.scale() * m_maxOutput)

        m_safetyHelper.feed()
    }
}

fun TalonSRX.configurePIDF(p: Double, i: Double, d: Double, power: Double, velocity: Double, wheelSize: Double, sensorUnitsPerRotation: Double, dev: FeedbackDevice) {
    configurePIDF(p, i, d, Maths.calculateFGain(power, velocity, wheelSize, sensorUnitsPerRotation))
    configSelectedFeedbackSensor(dev, 0, 10)
}

fun TalonSRX.configurePIDF(p: Double, i: Double, d: Double, power: Double, rpm: Double, sensorUnitsPerRotation: Double, dev: FeedbackDevice) {
    configurePIDF(p, i, d, Maths.calculateFGain(power, rpm, sensorUnitsPerRotation))
    configSelectedFeedbackSensor(dev, 0, 10)
}

fun TalonSRX.configurePIDF(p: Double, i: Double, d: Double, f: Double) {
    config_kP(0, p, 10)
    config_kI(0, i, 10)
    config_kD(0, d, 10)
    config_kF(0, f, 10)
}