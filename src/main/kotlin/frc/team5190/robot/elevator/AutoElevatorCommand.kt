package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command

class AutoElevatorCommand(val position: ElevatorPosition) : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    override fun initialize() {
        ElevatorSubsystem.set(ControlMode.Position, position.ticks)
    }

    override fun isFinished(): Boolean = ElevatorSubsystem.closedLoopErrorInches < 1
}

enum class ElevatorPosition(var ticks: Int) {
    SWITCH(ElevatorSubsystem.inchesToNativeUnits(18.0)),
    SCALE(ElevatorSubsystem.inchesToNativeUnits(60.0))
}