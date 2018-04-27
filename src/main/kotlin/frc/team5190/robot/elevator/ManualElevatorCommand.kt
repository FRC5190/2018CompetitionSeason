/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.elevator

import edu.wpi.first.wpilibj.command.Command

class ManualElevatorCommand : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    // Command never ends because it is the default command
    override fun isFinished() = false
}