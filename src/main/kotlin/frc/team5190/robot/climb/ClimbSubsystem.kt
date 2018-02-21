package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.util.MotorIDs

object ClimbSubsystem : Subsystem() {

    private val backWinchMaster = TalonSRX(MotorIDs.BACK_WINCH_MASTER)

    init {
        val backWinchSlave = TalonSRX(MotorIDs.BACK_WINCH_SLAVE)
        backWinchSlave.follow(backWinchMaster)
    }

    override fun initDefaultCommand() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

}

class WinchCommand : Command() {

    init {
        requires(ClimbSubsystem)
    }

    override fun isFinished(): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

}