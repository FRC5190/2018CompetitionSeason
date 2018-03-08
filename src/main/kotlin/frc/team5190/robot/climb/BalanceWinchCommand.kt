/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.PIDCommand
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.ClimbConstants

class BalanceWinchCommand : PIDCommand(0.2, 0.0, 0.0) {

    init {
        requires(ClimbSubsystem)
    }

    override fun initialize() {
        pidController.setInputRange(-180.0, 180.0)
        pidController.setOutputRange(-ClimbConstants.CORRECTION_OUTPUT, ClimbConstants.CORRECTION_OUTPUT)
        pidController.setAbsoluteTolerance(1.0)
        pidController.setContinuous(true)
        pidController.setpoint = 3.0
    }

    override fun returnPIDInput() = NavX.roll.toDouble()

    override fun usePIDOutput(output: Double) {
        ClimbSubsystem.frontWinchMotor.set(ControlMode.PercentOutput, (ClimbConstants.PEAK_OUTPUT - output * 2.0).coerceIn(0.0, ClimbConstants.PEAK_OUTPUT))
        ClimbSubsystem.backWinchMotor.set(ControlMode.PercentOutput, (ClimbConstants.PEAK_OUTPUT + output * 2.0).coerceIn(0.0, ClimbConstants.PEAK_OUTPUT))
    }

    override fun isFinished() = false

}