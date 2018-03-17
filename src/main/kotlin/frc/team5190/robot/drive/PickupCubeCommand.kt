/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.util.*
import frc.team5190.robot.vision.Vision

class PickupCubeCommand(private val outSpeed: Double = IntakeConstants.DEFAULT_SPEED,
                        private val maxDist: Double = 100.0) : PIDCommand(DriveConstants.TURN_P, DriveConstants.TURN_I, DriveConstants.TURN_D) {

    init {
        requires(DriveSubsystem)
    }

    override fun initialize() {

        DriveSubsystem.resetEncoders()

        setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-0.5, 0.5)
        pidController.setAbsoluteTolerance(5.0)
        pidController.setContinuous(true)

        setpoint = 0.0

        IntakeSubsystem.set(ControlMode.PercentOutput, outSpeed)
    }

    override fun returnPIDInput() = -Vision.tgtAngle.toDouble()

    override fun usePIDOutput(output: Double) = DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.3 + output, 0.3 - output, false)

    override fun isFinished() = IntakeSubsystem.isCubeIn || DriveSubsystem.falconDrive.allMasters.any {
        Maths.nativeUnitsToFeet(it.sensorCollection.quadraturePosition) > maxDist
    }
}