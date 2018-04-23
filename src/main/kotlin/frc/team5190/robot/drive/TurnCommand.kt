/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.sensors.Pigeon
import frc.team5190.robot.util.DriveConstants

class TurnCommand(val angle: Double) : PIDCommand(DriveConstants.TURN_P, DriveConstants.TURN_I, DriveConstants.TURN_D) {

    init {
        requires(DriveSubsystem)
    }

    override fun initialize() {
        setTimeout(2.5)

        setpoint = angle

        setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-1.0, 1.0)
        pidController.setAbsoluteTolerance(5.0)
        pidController.setContinuous(true)
    }

    override fun usePIDOutput(output: Double) = DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, -output, output)

    override fun returnPIDInput(): Double = Pigeon.correctedAngle

    // Time variable for isFinished method
    private var time = 0

    override fun isFinished(): Boolean {
        if (pidController.onTarget()) {
            time++
        } else {
            time = 0
        }
        return (time > 300 / 20) || isTimedOut
    }
}