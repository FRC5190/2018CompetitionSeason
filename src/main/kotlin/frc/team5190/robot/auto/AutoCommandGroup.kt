package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup

class AutoCommandGroup(initialPath: Paths) : CommandGroup() {
    init {
        this.addSequential(AutoCommand(initialPath))
        this.addSequential(TurnCommand(90.0))
    }
}