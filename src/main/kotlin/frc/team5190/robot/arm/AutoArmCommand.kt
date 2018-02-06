package frc.team5190.robot.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command

class AutoArmCommand(private val armPosition: ArmPosition) : Command() {

    init {
        requires(ArmSubsystem)
    }

    override fun initialize() {
        ArmSubsystem.set(ControlMode.MotionMagic, armPosition.ticks.toDouble())
    }

    override fun isFinished() = ArmSubsystem.closedLoopError < 150
}


enum class ArmPosition (val ticks: Int){
    BEHIND(2800 - 800), // When placing scale backwards
    UP(2600 - 800), // Arm is always up, basically where it starts in auto
    MIDDLE(1900 - 800), // Angled a little up to help placement on scale and switch
    DOWN(1668 - 800); // Lowest position, used for intaking the cube
}
