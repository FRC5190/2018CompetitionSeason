package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.OI
import frc.team5190.robot.util.*

object DriveTrain : Subsystem() {

    override fun initDefaultCommand() {
        this.defaultCommand = DriveCommand()
    }

    val frontLeft = TalonSRX(FRONT_LEFT)
    val frontRight = TalonSRX(FRONT_RIGHT)
    val rearLeft = TalonSRX(REAR_LEFT)
    val rearRight = TalonSRX(REAR_RIGHT)

    val navX = AHRS(SPI.Port.kMXP)

    init {

        DTRHelper.configurePIDF(frontLeft, 0.6, 0.0, 0.0, 1.0, HIGH_GEAR_MAX, WHEEL_RADIUS.toDouble(), TICKS_PER_ROTATION.toDouble(), FeedbackDevice.QuadEncoder)
        DTRHelper.configurePIDF(frontRight, 0.6, 0.0, 0.0, 1.0, HIGH_GEAR_MAX, WHEEL_RADIUS.toDouble(), TICKS_PER_ROTATION.toDouble(), FeedbackDevice.QuadEncoder)

        rearLeft.follow(frontLeft)
        rearRight.follow(frontRight)
    }

    fun driveWithXbox() {
        this.tankDrive(OI.getY(GenericHID.Hand.kLeft), OI.getY(GenericHID.Hand.kRight), ControlMode.PercentOutput)
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
}