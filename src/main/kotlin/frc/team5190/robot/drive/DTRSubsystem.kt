package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.OI
import frc.team5190.robot.util.Hardware
import frc.team5190.robot.util.MotorIDs

object DriveTrain : Subsystem() {

    val frontLeft = TalonSRX(MotorIDs.FRONT_LEFT)
    val frontRight = TalonSRX(MotorIDs.FRONT_RIGHT)

    private val rearLeft = TalonSRX(MotorIDs.REAR_LEFT)
    private val rearRight = TalonSRX(MotorIDs.REAR_RIGHT)

    private val navX = AHRS(SPI.Port.kMXP)

    init {

        DTRHelper.configurePIDF(frontLeft, 0.0, 0.0, 0.0, 1.0, rpm = Hardware.MAX_RPM.toDouble(),
                sensorUnitsPerRotation = Hardware.NATIVE_UNITS_PER_ROTATION.toDouble(), dev = FeedbackDevice.QuadEncoder)

        DTRHelper.configurePIDF(frontRight, 0.0, 0.0, 0.0, 1.0, rpm = Hardware.MAX_RPM.toDouble(),
                sensorUnitsPerRotation = Hardware.NATIVE_UNITS_PER_ROTATION.toDouble(), dev = FeedbackDevice.QuadEncoder)

        frontLeft.configMotionProfileTrajectoryPeriod(10, 10)
        frontRight.configMotionProfileTrajectoryPeriod(10, 10)

        frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 50, 10)
        frontRight.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 10)

        frontLeft.inverted = true
        rearLeft.inverted = true

        frontLeft.configNeutralDeadband(0.04, 10)
        frontRight.configNeutralDeadband(0.04, 10)

        rearLeft.follow(frontLeft)
        rearRight.follow(frontRight)
    }

    fun driveWithXbox() {
        this.tankDrive(OI.getLeftY(), OI.getRightY(), ControlMode.Velocity)
    }

    fun tankDrive(leftOutput: Double, rightOutput: Double, mode: ControlMode) {
        when (mode) {
            ControlMode.PercentOutput -> {
                frontLeft.set(mode, leftOutput)
                frontRight.set(mode, rightOutput)

                println("Left: " + frontLeft.sensorCollection.quadraturePosition + ", Right: " + frontRight.sensorCollection.quadraturePosition)
            }
            ControlMode.Velocity -> {
                frontLeft.set(mode, leftOutput * Hardware.MAX_NATIVE_UNITS_PER_100_MS)
                frontRight.set(mode, rightOutput * Hardware.MAX_NATIVE_UNITS_PER_100_MS)
            }
            else -> {
                println("Learn to dab sir.")
            }
        }
    }

    override fun initDefaultCommand() {
        this.defaultCommand = DriveCommand()
    }

    fun reset() {
        frontLeft.set(ControlMode.PercentOutput, 0.0)
        frontRight.set(ControlMode.PercentOutput, 0.0)
        navX.angleAdjustment = 90.0
    }
}