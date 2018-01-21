package frc.team5190.robot.elevator

import edu.wpi.first.wpilibj.command.Command

class AutoElevatorCommand(val pos: ElevatorPosition) : Command() {
    init {
        requires(ElevatorSubsystem)
    }

    override fun execute() {
        ElevatorSubsystem.moveToPosition(pos.ticks)
    }

    override fun isFinished() = false
}

enum class ElevatorPosition(var ticks: Int) {
    SWITCH(4000), SCALE(18000)
}