package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.OI
import frc.team5190.robot.util.Hardware
import frc.team5190.robot.util.Hardware.HIGH_GEAR_MAX
import frc.team5190.robot.util.MotorIDs

object DriveTrain : Subsystem() {

    val frontLeft = TalonSRX(MotorIDs.FRONT_LEFT)
    val frontRight = TalonSRX(MotorIDs.FRONT_RIGHT)

    private val rearLeft = TalonSRX(MotorIDs.REAR_LEFT)
    private val rearRight = TalonSRX(MotorIDs.REAR_RIGHT)

    private val navX = AHRS(SPI.Port.kMXP)

    init {

        DTRHelper.configurePIDF(frontLeft, 0.6, 0.0, 0.0, 1.0, Hardware.HIGH_GEAR_MAX, Hardware.WHEEL_RADIUS.toDouble(),
                Hardware.TICKS_PER_ROTATION.toDouble(), FeedbackDevice.QuadEncoder)

        DTRHelper.configurePIDF(frontRight, 0.6, 0.0, 0.0, 1.0, Hardware.HIGH_GEAR_MAX, Hardware.WHEEL_RADIUS.toDouble(),
                Hardware.TICKS_PER_ROTATION.toDouble(), FeedbackDevice.QuadEncoder)

        rearLeft.follow(frontLeft)
        rearRight.follow(frontRight)
    }

    fun driveWithXbox() {
        this.tankDrive(OI.getLeftY(), OI.getRightY(), ControlMode.PercentOutput)
    }

    private fun tankDrive(leftOutput: Double, rightOutput: Double, mode: ControlMode) {
        when (mode) {
            ControlMode.PercentOutput -> {
                frontLeft.set(mode, leftOutput)
                frontRight.set(mode, rightOutput)
            }
            ControlMode.Velocity -> {
                frontLeft.set(mode, leftOutput * HIGH_GEAR_MAX)
                frontRight.set(mode, rightOutput * HIGH_GEAR_MAX)
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