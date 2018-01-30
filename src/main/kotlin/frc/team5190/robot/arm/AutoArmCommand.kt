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

    override fun isFinished() = ArmSubsystem.closedLoopError < 50
}


enum class ArmPosition (val ticks: Int){
    BEHIND(1101), // When placing scale backwards
    UP(971), // Arm is always up, basically where it starts in auto
    MIDDLE(800), // Angled a little up to help placement on scale and switch
    DOWN(720); // Lowest position, used for intaking the cube
}
