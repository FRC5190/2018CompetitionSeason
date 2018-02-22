/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan S, Prateek M
 */

package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox


class ManualArmCommand : Command() {

    init {
        requires(ArmSubsystem)
    }

    override fun execute() {
        when {
            MainXbox.yButton -> ArmSubsystem.set(ControlMode.PercentOutput, 0.7)
            MainXbox.bButton -> ArmSubsystem.set(ControlMode.PercentOutput, -0.4)

            MainXbox.yButtonReleased -> ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())
            MainXbox.bButtonReleased -> ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())
        }
    }

    override fun isFinished() = false
}
