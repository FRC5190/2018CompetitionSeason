package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import kotlin.math.absoluteValue

class AutoArmCommand(private val armPosition: ArmPosition) : Command() {

    init {
        requires(ArmSubsystem)
    }

    override fun initialize() {
        ArmSubsystem.set(ControlMode.MotionMagic, armPosition.ticks.toDouble())
    }

    override fun isFinished() = (ArmSubsystem.currentPosition - armPosition.ticks).absoluteValue  < 50
}


enum class ArmPosition (val ticks: Int){
    BEHIND(800), // When placing scale backwards
    UP(600), // Arm is always up, basically where it starts in auto
    MIDDLE(425), // Angled a little up to help placement on scale and switch
    DOWN(370); // Lowest position, used for intaking the cube
}
