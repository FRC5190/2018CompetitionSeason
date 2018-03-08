/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.util.*

object ClimbSubsystem : Subsystem() {

    val hookSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.HOOK)

    internal val frontWinchMotor = TalonSRX(MotorIDs.FRONT_WINCH_MASTER).apply { configPeakOutput(ClimbConstants.PEAK_OUTPUT, -ClimbConstants.PEAK_OUTPUT, TIMEOUT) }
    internal val backWinchMotor = TalonSRX(MotorIDs.BACK_WINCH_MASTER).apply { configPeakOutput(ClimbConstants.PEAK_OUTPUT, -ClimbConstants.PEAK_OUTPUT, TIMEOUT) }

    init {
        with(TalonSRX(MotorIDs.BACK_WINCH_SLAVE)) {
            follow(backWinchMotor)
        }
    }

    var climbState = false

    override fun initDefaultCommand() {
        defaultCommand = IdleClimbCommand()
    }

    override fun periodic() {
        Controls.climbSubsystem()
    }
}