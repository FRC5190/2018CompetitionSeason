/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.arm

import edu.wpi.first.wpilibj.command.Command


class ManualArmCommand : Command() {

    init {
        requires(ArmSubsystem)
    }

    // Checks command for completion
    override fun isFinished() = false
}
