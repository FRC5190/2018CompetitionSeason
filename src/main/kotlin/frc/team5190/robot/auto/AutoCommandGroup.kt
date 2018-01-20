package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup

class AutoCommandGroup(helper: AutoHelper) : CommandGroup() {
    init {
        this.addSequential(AutoCommand(helper))
        this.addSequential(TurnCommand(90.0))
    }
}