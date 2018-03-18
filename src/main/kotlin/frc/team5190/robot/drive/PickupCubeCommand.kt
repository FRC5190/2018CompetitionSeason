/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.IntakeConstants
import frc.team5190.robot.util.Maths
import frc.team5190.robot.vision.Vision

class PickupCubeCommand(private val outSpeed: Double = -IntakeConstants.DEFAULT_SPEED,
                        private val maxDist: Double = 100.0) : PIDCommand(0.01, 0.0, 0.0) {

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

    override fun returnPIDInput() = if(Vision.isTgtVisible == 1L) Vision.tgtAngleAbsolute - NavX.angle else 0.0

    override fun usePIDOutput(output: Double) {

        println("Vision Angle: ${returnPIDInput()}, PID Output: $output")

        val speed: Double = if (output < 0.05) 0.25 else 0.25
        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, speed - output, speed + output, false)
    }

    override fun isFinished() = IntakeSubsystem.isCubeIn || DriveSubsystem.falconDrive.allMasters.any {
        Maths.nativeUnitsToFeet(it.sensorCollection.quadraturePosition) > maxDist
    }
}