/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.MainXbox

/**
 * Command that moves the arm using input from the controller
 */
class ManualArmCommand : Command() {

    init {
        requires(ArmSubsystem)
    }

    /**
     * Executes periodically
     */
    override fun execute() {
        when {
            MainXbox.yButton -> ArmSubsystem.set(ControlMode.PercentOutput, 0.7)
            MainXbox.bButton -> ArmSubsystem.set(ControlMode.PercentOutput, -0.4)

            MainXbox.yButtonReleased -> ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())
            MainXbox.bButtonReleased -> ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())
        }
    }

    /**
     * Command never finishes because it's the default command
     */
    override fun isFinished() = false
}
