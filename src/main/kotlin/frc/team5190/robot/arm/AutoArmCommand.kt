package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command

class AutoArmCommand(private val armPosition: ArmPosition) : Command() {

    init {
        requires(ArmSubsystem)
    }

    override fun initialize() {
        ArmSubsystem.set(ControlMode.Position, armPosition.ticks.toDouble())
    }

    override fun isFinished() = ArmSubsystem.closedLoopError < 400
}


enum class ArmPosition (val ticks: Int){
    UP(0), MIDDLE(0), DOWN(0)
}