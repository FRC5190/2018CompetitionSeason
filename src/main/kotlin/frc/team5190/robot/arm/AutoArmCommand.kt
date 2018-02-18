package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import kotlin.math.absoluteValue

class AutoArmCommand(armPosition: ArmPosition) : Command() {

    private val armPosition = armPosition.ticks

    init {
        requires(ArmSubsystem)
    }

    override fun initialize() {
        ArmSubsystem.set(ControlMode.MotionMagic, armPosition.toDouble())
    }

    override fun isFinished() = (ArmSubsystem.currentPosition - armPosition).absoluteValue < 150
}
