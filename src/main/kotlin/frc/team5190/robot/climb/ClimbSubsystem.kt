/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.util.*

object ClimbSubsystem : Subsystem() {

    private val masterClimbMotor = TalonSRX(MotorIDs.WINCH_MASTER)

    init {
        with(masterClimbMotor){
            configPeakOutput(ClimbConstants.PEAK_OUTPUT, -ClimbConstants.PEAK_OUTPUT, TIMEOUT)

            configPID(0, 1.0, 0.0, 0.0, TIMEOUT)
            configMotionCruiseVelocity(1000000, TIMEOUT)
            configMotionAcceleration(4096, TIMEOUT)
        }
        with(TalonSRX(MotorIDs.WINCH_SLAVE)) {
            follow(masterClimbMotor)
        }
    }

    var climbState = false

    fun set(controlMode: ControlMode, output: Double) {
        masterClimbMotor.set(controlMode, output)
    }

    override fun initDefaultCommand() {
        defaultCommand = IdleClimbCommand()
    }

    override fun periodic() {
        Controls.climbSubsystem()
    }
}