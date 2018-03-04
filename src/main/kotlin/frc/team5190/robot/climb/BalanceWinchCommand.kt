/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.sensors.NavX

class BalanceWinchCommand : PIDCommand(0.02, 0.0, 0.0) {

    init {
        requires(ClimbSubsystem)
    }

    override fun initialize() {
        pidController.setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-0.2, 0.2)
        pidController.setAbsoluteTolerance(1.0)
        pidController.setContinuous(true)
        pidController.setpoint = ClimbSubsystem.gyropitch - 5.0
    }

    override fun returnPIDInput() = NavX.pitch.toDouble()

    override fun usePIDOutput(output: Double) {
        ClimbSubsystem.frontWinchMotor.set(ControlMode.PercentOutput, (0.4 - output).coerceAtLeast(0.0))
        ClimbSubsystem.backWinchMotor.set(ControlMode.PercentOutput, (0.4 + output).coerceAtLeast(0.0))
    }

    override fun isFinished() = false

}