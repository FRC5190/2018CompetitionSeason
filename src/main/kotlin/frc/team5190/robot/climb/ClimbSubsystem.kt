/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.climb

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.util.*

object ClimbSubsystem : Subsystem() {

    // Climb master talon
    val masterClimbMotor = TalonSRX(MotorIDs.WINCH_MASTER)

    init {
        with(masterClimbMotor) {
            // Motor and sensor inversion
            inverted = false
            setSensorPhase(false)

            // Peak output
            configPeakOutput(ClimbConstants.PEAK_OUTPUT, -ClimbConstants.PEAK_OUTPUT, TIMEOUT)

            // PID and Motion Magic
            configPID(0, 2.0, 0.0, 0.0, TIMEOUT)
            configMotionCruiseVelocity(1000000, TIMEOUT)
            configMotionAcceleration(12000, TIMEOUT)

            // Limit switch
            configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TIMEOUT)
            overrideLimitSwitchesEnable(true)
        }
        val slaveClimbMotor = TalonSRX(MotorIDs.WINCH_SLAVE).apply {
            follow(masterClimbMotor)
            inverted = false
        }
        arrayOf(masterClimbMotor, slaveClimbMotor).forEach {
            // Current Limiting
            it.configContinuousCurrentLimit(40, TIMEOUT)
            it.configPeakCurrentDuration(0, TIMEOUT)
            it.configPeakCurrentLimit(0, TIMEOUT)
            it.enableCurrentLimit(true)
        }
    }

    // Whether the robot is ready to climb / is climbing
    var climbState = false

    // Sets motor output
    fun set(controlMode: ControlMode, output: Double) {
        masterClimbMotor.set(controlMode, output)
    }

    // Default command
    override fun initDefaultCommand() {
        defaultCommand = IdleClimbCommand()
    }

    // Periodic 50hz loop
    override fun periodic() {
        Controls.climbSubsystem()

        SmartDashboard.putNumber("Winch Encoder", masterClimbMotor.getSelectedSensorPosition(0).toDouble())
        SmartDashboard.putNumber("Winch Percent", masterClimbMotor.motorOutputPercent)
    }
}