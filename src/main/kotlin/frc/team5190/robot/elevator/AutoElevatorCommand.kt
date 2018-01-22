package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command

class AutoElevatorCommand(val pos: ElevatorPosition) : Command() {
    init {
        requires(ElevatorSubsystem)
    }

    override fun initialize() {
        ElevatorSubsystem.set(ControlMode.Position, pos.ticks)
    }

    override fun isFinished(): Boolean = ElevatorSubsystem.closedLoopErrorInches < 3
}

enum class ElevatorPosition(var ticks: Int) {
    SWITCH(ElevatorSubsystem.inchesToNativeUnits(9.0)),
    SCALE(ElevatorSubsystem.inchesToNativeUnits(40.0))
}