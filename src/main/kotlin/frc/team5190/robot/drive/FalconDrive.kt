/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import frc.team5190.robot.util.*

/**
 * Custom FalconDrive object that extends Differential Drive
 */
class FalconDrive(val leftMotors: List<WPI_TalonSRX>,
                  val rightMotors: List<WPI_TalonSRX>) : DifferentialDrive(leftMotors[0], rightMotors[0]), {

    // Values for the left side of the DriveTrain
    val leftMaster = leftMotors[0]
    private val leftSlaves = leftMotors.subList(1, leftMotors.size)

    // Values for the right side of the DriveTrain
    val rightMaster = rightMotors[0]
    private val rightSlaves = rightMotors.subList(1, rightMotors.size)

    // Values for all the master motors of the DriveTrain
    private val allMasters = listOf(leftMaster, rightMaster)

    // Values for all the motors of the Drive Train
    private val allMotors = listOf(*leftMotors.toTypedArray(), *rightMotors.toTypedArray())

    /**
     * Sets some initial values when the FalconDrive object is initialized.
     */
    init {
        reset()
    }

    /**
     * Reset the drive train subsystem
     * Call this when initializing  autonomous and teleop
     * Resets all motors, their directions, and encoders
     */
    private fun reset() {
        leftMotors.forEach { it.inverted = false }
        rightMotors.forEach { it.inverted = true }

        leftSlaves.forEach { it.follow(leftMaster) }
        rightSlaves.forEach { it.follow(rightMaster) }

        allMasters.forEach {
            it.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10)
            it.setSelectedSensorPosition(0, 0, 10)
        }

        allMotors.forEach {
            it.setSensorPhase(true)
            it.setNeutralMode(NeutralMode.Brake)

            // TODO: Need to configure current limits
        }
    }

    internal fun autoReset() {
        this.reset()

        allMasters.forEach {
            it.configPIDF(0, 0.7, 0.0, 0.0, 1.0, Hardware.MAX_RPM.toDouble(), Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
            it.selectProfileSlot(0, 0)
            it.configMotionProfileTrajectoryPeriod(10, 10)
            it.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10)
        }
    }

    internal fun teleopReset() {
        this.reset()
        allMasters.forEach {
            it.configPIDF(1, 0.0, 0.0, 0.0, 0.0, Hardware.MAX_RPM.toDouble(), Hardware.NATIVE_UNITS_PER_ROTATION.toDouble())
            it.clearMotionProfileTrajectories()
            it.selectProfileSlot(1, 0)
        }
    }

    val leftEncoderPosition
        get() = leftMaster.getSelectedSensorPosition(0)

    val rightEncoderPosition
        get() = rightMaster.getSelectedSensorPosition(0)

    fun feedSafety() {
        m_safetyHelper.feed()
    }

    /**
     * Drives the motors in tank drive motion.
     * @param controlMode The control mode in which to drive.
     * @param _leftSpeed The speed at which the left side of the DriveTrain should move.
     * @param _rightSpeed The speed at which the right side of the DriveTrain should move.
     * @param squaredInputs Decides if the inputs should be squared.
     */
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

    /**
     * Drives the motors in curvature drive motion.
     * @param controlMode The control mode in which to drive.
     * @param xSpeed The speed of the X axis.
     * @param zRotation The speed of the rotation along the Z axis.
     * @param isQuickTurn Decides if quick turn is enabled or disabled.
     */
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