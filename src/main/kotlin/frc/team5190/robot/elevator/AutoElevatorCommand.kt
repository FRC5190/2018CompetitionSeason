package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import kotlin.math.absoluteValue

class AutoElevatorCommand(val position: ElevatorPosition) : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    override fun initialize() {
        ElevatorSubsystem.set(ControlMode.MotionMagic, position.ticks)
    }

    override fun isFinished() = (ElevatorSubsystem.currentPosition - position.ticks).absoluteValue < ElevatorSubsystem.inchesToNativeUnits(2.0)
}

enum class ElevatorPosition(var ticks: Int) {
    SWITCH(ElevatorSubsystem.inchesToNativeUnits(18.0)),
    SCALE(ElevatorSubsystem.inchesToNativeUnits(60.0)),
    INTAKE(500)
}