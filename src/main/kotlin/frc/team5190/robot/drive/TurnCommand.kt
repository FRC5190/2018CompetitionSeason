/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.DriveConstants
import frc.team5190.robot.vision.Vision

/**
 * Command that turns the robot to a certain angle
 * @param angle Angle to turn to in degrees
 * @param visionCheck Whether to use vision for cube detection
 * @param tolerance Tolerance
 */
class TurnCommand(val angle: Double, val visionCheck: Boolean = false, val tolerance: Double = 0.0) : PIDCommand(DriveConstants.TURN_P, DriveConstants.TURN_I, DriveConstants.TURN_D) {

    init {
        requires(DriveSubsystem)
    }

    /**
     * Initializes the command
     */
    override fun initialize() {
        setTimeout(2.5)

        when (visionCheck) {
            false -> setpoint = angle
            true -> {
                when (Vision.isTgtVisible == 1L) {
                    false -> {
                        println("Vision subsystem did not find any target object")
                        setpoint = angle
                    }
                    true -> {
                        val x = NavX.pidGet()                   // current absolute angle
                        val y = x + (Vision.tgtAngle + Vision.rawAngle) / 2.0    // Vision absolute angle
                        // (y - angle) is correction and it should be less than tolerance
                        setpoint = if (Math.abs(y - angle) < tolerance) {
                            println("Vision subsystem corrected $angle to $y")
                            y
                        } else {
                            println("Vision subsystem found object at $y instead of $angle, and it was outside tolerance range")
                            angle
                        }
                    }
                }
            }
        }

        setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-1.0, 1.0)
        pidController.setAbsoluteTolerance(5.0)
        pidController.setContinuous(true)
    }

    /**
     * Uses the output of the PID controller to control the DriveTrain
     */
    override fun usePIDOutput(output: Double) = DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, output, -output)

    /**
     * Input from the PID comes from the NavX
     */
    override fun returnPIDInput(): Double = NavX.pidGet()

    // Time variable for isFinished method
    private var time = 0

    /**
     * Checks if the robot is at the specified angle
     */
    override fun isFinished(): Boolean {
        if (pidController.onTarget()) {
            time++
        } else {
            time = 0
        }
        return (time > 300 / 20) || isTimedOut
    }
}