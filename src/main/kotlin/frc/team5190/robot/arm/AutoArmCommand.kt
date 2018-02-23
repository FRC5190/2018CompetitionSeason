/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import kotlin.math.absoluteValue

/**
 * Command that moves the arm to the desired location using Motion Magic
 * @param armPosition Arm position
 */
class AutoArmCommand(armPosition: ArmPosition) : Command() {

    // Arm position in Native Units
    private val armPosition = armPosition.ticks

    init {
        requires(ArmSubsystem)
    }

    /**
     * Initializes the command
     */
    override fun initialize() {
        ArmSubsystem.set(ControlMode.MotionMagic, armPosition.toDouble())
    }

    /**
     * Checks if the command has finished executing
     */
    override fun isFinished() = (ArmSubsystem.currentPosition - armPosition).absoluteValue < 150
}
