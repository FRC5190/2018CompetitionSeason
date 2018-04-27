/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.Pigeon
import frc.team5190.robot.util.IntakeConstants
import frc.team5190.robot.util.Maths
import kotlin.math.absoluteValue

class PickupCubeCommand(private val inSpeed: Double = IntakeConstants.DEFAULT_IN_SPEED,
                        private val maxDist: Double = 100.0, val visionCheck: Boolean = true) : PIDCommand(0.01, 0.0, 0.0) {

    init {
        requires(DriveSubsystem)
        requires(IntakeSubsystem)
    }

    // Initializes the command
    override fun initialize() {

        // This command used to integrate Vision. We scrapped the idea because we didn't find it useful. This is the remnant class.
        // All this command does now is drive forward until it has a cube

        DriveSubsystem.resetEncoders()

        setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-0.5, 0.5)
        pidController.setAbsoluteTolerance(5.0)
        pidController.setContinuous(true)

        setpoint = Pigeon.correctedAngle

        IntakeSubsystem.intakeSolenoid.set(false)
        IntakeSubsystem.set(ControlMode.PercentOutput, -(inSpeed.absoluteValue))
    }

    // PID input value
    override fun returnPIDInput() = Pigeon.correctedAngle

    // Use PID calculated output
    override fun usePIDOutput(output: Double) {
        val speed = 0.25
        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, speed - output, speed + output, false)
    }

    // Checks command for completion
    override fun isFinished() = IntakeSubsystem.isCubeIn || DriveSubsystem.falconDrive.allMasters.any {
        Maths.nativeUnitsToFeet(it.sensorCollection.quadraturePosition) > maxDist
    }
}