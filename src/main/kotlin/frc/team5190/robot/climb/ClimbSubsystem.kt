/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.util.*

object ClimbSubsystem : Subsystem() {

    val masterClimbMotor = TalonSRX(MotorIDs.WINCH_MASTER)

    init {
        with(masterClimbMotor) {
            inverted = !DriveConstants.IS_RACE_ROBOT
            setSensorPhase(!DriveConstants.IS_RACE_ROBOT)

            configPeakOutput(ClimbConstants.PEAK_OUTPUT, -ClimbConstants.PEAK_OUTPUT, TIMEOUT)

            configPID(0, 5.0, 0.0, 0.0, TIMEOUT)
            configMotionCruiseVelocity(1000000, TIMEOUT)
            configMotionAcceleration(4096, TIMEOUT)
        }
        with(TalonSRX(MotorIDs.WINCH_SLAVE)) {
            follow(masterClimbMotor)
            inverted =  !DriveConstants.IS_RACE_ROBOT
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

        SmartDashboard.putNumber("Winch Encoder", masterClimbMotor.getSelectedSensorPosition(0).toDouble())
        SmartDashboard.putNumber("Winch Percent", masterClimbMotor.motorOutputPercent)
    }
}