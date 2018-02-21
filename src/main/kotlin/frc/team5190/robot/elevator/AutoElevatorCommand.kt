package frc.team5190.robot.elevator

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import kotlin.math.absoluteValue

class AutoElevatorCommand(val position: ElevatorPosition) : Command() {

    init {
        requires(ElevatorSubsystem)
    }

    override fun initialize() {
        ElevatorSubsystem.set(ControlMode.MotionMagic, position.ticks.toDouble())
    }

    override fun isFinished() = (ElevatorSubsystem.currentPosition - position.ticks).absoluteValue < ElevatorSubsystem.inchesToNativeUnits(1.0)
}
