/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.arm

import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.util.Controls

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
    override fun execute() = Controls.armSubsystem()

    /**
     * Command never finishes because it's the default command
     */
    override fun isFinished() = false
}
