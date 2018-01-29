package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import kotlin.math.absoluteValue

class AutoElevatorCommand(val position: ElevatorPosition) : Command() {

    private var time: Long = 0

    init {
        requires(ElevatorSubsystem)
    }

    override fun initialize() {
        ElevatorSubsystem.set(ControlMode.MotionMagic, position.ticks)
    }

    override fun end() {
        println("HAS FINISHED")
        time = 0
    }

    override fun isFinished(): Boolean {
        when(ElevatorSubsystem.closedLoopErrorInches.absoluteValue < 1){
            true -> time++
            else -> time = 0
        }
        return time > 20
    }
}

enum class ElevatorPosition(var ticks: Int) {
    SWITCH(ElevatorSubsystem.inchesToNativeUnits(18.0)),
    SCALE(ElevatorSubsystem.inchesToNativeUnits(60.0)),
    INTAKE(0)
}