package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.util.MotorIDs
import frc.team5190.robot.util.configPeakOutput

object WinchSubsystem : Subsystem() {

    internal val frontWinchMotor = TalonSRX(MotorIDs.FRONT_WINCH_MASTER).apply { configPeakOutput(0.6, -0.6, 10) }
    internal val backWinchMotor = TalonSRX(MotorIDs.BACK_WINCH_MASTER).apply { configPeakOutput(0.6, -0.6, 10) }

    var winchState = false

    init {
        with(TalonSRX(MotorIDs.BACK_WINCH_SLAVE)) {
            follow(backWinchMotor)
        }
    }

    override fun initDefaultCommand() {
        this.defaultCommand = object : Command(){
            init {
                requires(WinchSubsystem)
            }
            override fun initialize() {
                WinchSubsystem.backWinchMotor.set(ControlMode.PercentOutput, 0.0)
                WinchSubsystem.frontWinchMotor.set(ControlMode.PercentOutput, 0.0)
            }
            override fun isFinished() = false
        }
    }

    override fun periodic() {
        if (MainXbox.startButtonPressed) {
            winchState = true
            WinchCommand().start()
        }
    }
}

class WinchCommand : Command() {

    init {
        requires(WinchSubsystem)
    }

    override fun execute() {
        if (!MainXbox.getBumper(GenericHID.Hand.kLeft)) WinchSubsystem.frontWinchMotor.set(ControlMode.PercentOutput, MainXbox.getTriggerAxis(GenericHID.Hand.kLeft))
        if (!MainXbox.getBumper(GenericHID.Hand.kRight)) WinchSubsystem.backWinchMotor.set(ControlMode.PercentOutput, MainXbox.getTriggerAxis(GenericHID.Hand.kRight))

        if (MainXbox.getBumper(GenericHID.Hand.kLeft)) WinchSubsystem.frontWinchMotor.set(ControlMode.PercentOutput, -0.3)
        if (MainXbox.getBumper(GenericHID.Hand.kRight)) WinchSubsystem.backWinchMotor.set(ControlMode.PercentOutput, -0.3)
    }


    override fun isFinished() = false
}