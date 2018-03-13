/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.DriveConstants
import frc.team5190.robot.util.IntakeConstants
import frc.team5190.robot.vision.Vision

class PickupCubeCommand(val angle: Double = 0.0) : PIDCommand(DriveConstants.TURN_P, DriveConstants.TURN_I, DriveConstants.TURN_D) {

    init {
        requires(DriveSubsystem)
    }

    override fun initialize() {

        DriveSubsystem.resetEncoders()

        setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-0.5, 0.5)
        pidController.setAbsoluteTolerance(5.0)
        pidController.setContinuous(true)

        setpoint = if (Vision.isTgtVisible == 1L) Vision.tgtAngle else angle
    }

    override fun execute() {
        if (Vision.isTgtVisible == 1L) {
            setpoint = Vision.tgtAngle
        }
    }

    override fun returnPIDInput() = NavX.angle

    override fun usePIDOutput(output: Double) = DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.5 + output, 0.5 - output, false)

    override fun isFinished() = IntakeSubsystem.amperage > IntakeConstants.AMP_THRESHOLD
}