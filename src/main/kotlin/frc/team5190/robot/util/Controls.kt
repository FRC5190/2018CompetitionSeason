/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.util

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import frc.team5190.robot.*
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.climb.ClimbSubsystem
import frc.team5190.robot.climb.WinchCommand
import frc.team5190.robot.drive.*
import frc.team5190.robot.elevator.*
import frc.team5190.robot.intake.*
import kotlin.math.absoluteValue
import kotlin.math.pow

object Controls {

    private val driveMode = ControlMode.PercentOutput

    private var teleIntake = false
    private var triggerState = false
    private var autoShiftGear = Gear.HIGH

    fun driveSubsystem() {
        when {
            DriveSubsystem.controlMode == DriveMode.ARCADE -> DriveSubsystem.falconDrive.arcadeDrive(-MainXbox.getLeftY(), MainXbox.getLeftX())
            DriveSubsystem.controlMode == DriveMode.CURVE -> DriveSubsystem.falconDrive.curvatureDrive(driveMode, -MainXbox.getLeftY(), MainXbox.getLeftX(), MainXbox.xButton)
            DriveSubsystem.controlMode == DriveMode.TANK -> when {
                DriveSubsystem.controller == "Bongo" -> DriveSubsystem.falconDrive.tankDrive(driveMode, Bongos.getLeftBongoSpeed(), Bongos.getRightBongoSpeed())
                else -> DriveSubsystem.falconDrive.tankDrive(driveMode, -MainXbox.getLeftY(), -MainXbox.getRightY())
            }
        }

        if (Robot.INSTANCE!!.isOperatorControl) {
            // Auto Shift Logic
            val speed = DriveSubsystem.falconDrive.allMasters.map { Maths.nativeUnitsPer100MsToFeetPerSecond(it.getSelectedSensorVelocity(0).absoluteValue) }.average()
            when {
                speed > DriveConstants.AUTO_SHIFT_HIGH_THRESHOLD -> autoShiftGear = Gear.HIGH
                speed < DriveConstants.AUTO_SHIFT_LOW_THRESHOLD -> autoShiftGear = Gear.LOW
            }
            DriveSubsystem.falconDrive.gear = if (MainXbox.aButton) autoShiftGear else Gear.HIGH
        } else {
            autoShiftGear = Gear.HIGH
        }

    }

    fun intakeSubsystem() {

        val climbState = ClimbSubsystem.climbState


        when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) && !climbState -> {
                IntakeCommand(IntakeDirection.IN).start()
                teleIntake = true
            }
            MainXbox.getTriggerAxis(GenericHID.Hand.kLeft) >= 0.1 && !climbState -> {
                IntakeCommand(IntakeDirection.OUT, speed = MainXbox.getTriggerAxis(GenericHID.Hand.kLeft).pow(2.0) * 0.65).start()
                teleIntake = true
            }
            teleIntake -> {
                IntakeSubsystem.currentCommand?.cancel()
                teleIntake = false
            }
        }
    }

    fun armSubsystem() {
        when {
            MainXbox.yButton -> ArmSubsystem.set(ControlMode.PercentOutput, 0.4)
            MainXbox.bButton -> ArmSubsystem.set(ControlMode.PercentOutput, -0.3)

            MainXbox.yButtonReleased -> if (ElevatorSubsystem.closedLpControl) {
                ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition + 50.0)
            } else ArmSubsystem.set(ControlMode.PercentOutput, 0.0)
            MainXbox.bButtonReleased -> if (ElevatorSubsystem.closedLpControl) {
                ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())
            } else ArmSubsystem.set(ControlMode.PercentOutput, 0.0)
        }
    }

    private var lastPov = -1

    fun elevatorSubsystem() {

        if (MainXbox.getStickButtonPressed(GenericHID.Hand.kRight)) {
            if (ElevatorSubsystem.closedLpControl) {
                ElevatorSubsystem.disableSensorControl()
                ArmSubsystem.disableSensorControl()
            } else {
                ElevatorSubsystem.enableSensorControl()
                ArmSubsystem.enableSensorControl()
            }
        }

        when {
            MainXbox.getTriggerPressed(GenericHID.Hand.kRight) && !ClimbSubsystem.climbState -> {
                ElevatorSubsystem.currentCommand?.cancel()
                val motorOut = 0.55
                ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.ACTIVE_PEAK_OUT
                ElevatorSubsystem.set(ControlMode.PercentOutput, motorOut)
                triggerState = true
            }
            triggerState -> {
                ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT
                if (ElevatorSubsystem.closedLpControl) {
                    ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition.toDouble())
                } else ElevatorSubsystem.set(ControlMode.PercentOutput, 0.0)
                triggerState = false
            }
        }
        when {
            MainXbox.getBumper(GenericHID.Hand.kRight) && !ClimbSubsystem.climbState -> {
                ElevatorSubsystem.currentCommand?.cancel()
                ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.ACTIVE_PEAK_OUT
                val motorOut = -0.3
                ElevatorSubsystem.set(ControlMode.PercentOutput, motorOut)
            }
            MainXbox.getBumperReleased(GenericHID.Hand.kRight) && !ClimbSubsystem.climbState -> {
                ElevatorSubsystem.peakElevatorOutput = ElevatorConstants.IDLE_PEAK_OUT

                if (ElevatorSubsystem.closedLpControl) {
                    ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition.toDouble())
                } else ElevatorSubsystem.set(ControlMode.PercentOutput, 0.0)

            }
        }

        if (ClimbSubsystem.climbState) return

        val pov = MainXbox.pov
        if (lastPov != pov && ElevatorSubsystem.closedLpControl) {
            when (pov) {
            // Up - Scale
                0 -> ElevatorPresetCommand(ElevatorPreset.SCALE)
            // Right - Switch
                90 -> ElevatorPresetCommand(ElevatorPreset.SWITCH)
            // Down - Intake
                180 -> ElevatorPresetCommand(ElevatorPreset.INTAKE)
            // Left - Scale Backwards
                270 -> ElevatorPresetCommand(ElevatorPreset.BEHIND_LIDAR)
                else -> null
            }?.start()
        }
        lastPov = pov
    }

    fun climbSubsystem() {

        if (MainXbox.backButtonPressed) {
            ClimbSubsystem.climbState = true
            if(ElevatorSubsystem.closedLpControl) {
                commandGroup {
                    addParallel(AutoArmCommand(ArmPosition.ALL_UP))
                    addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                }.start()
            }
            WinchCommand().start()
        }

        if (MainXbox.startButtonPressed) {
            ClimbSubsystem.climbState = false
            ClimbSubsystem.currentCommand?.cancel()
        }
    }

    private var winchMoving = false

    fun winchSubsystem() {
        val winchSpeed = MainXbox.getRightY().takeIf { it.absoluteValue > 0.1 && DriveSubsystem.controlMode == DriveMode.CURVE }
        winchSpeed?.let {
            ClimbSubsystem.set(ControlMode.PercentOutput, -it * ClimbConstants.PEAK_OUTPUT)
            winchMoving = true
        }
        if(winchSpeed == null && winchMoving){
            if(ElevatorSubsystem.closedLpControl){
                ClimbSubsystem.set(ControlMode.MotionMagic, ClimbSubsystem.masterClimbMotor.getSelectedSensorPosition(0).toDouble())
            }else {
                ClimbSubsystem.set(ControlMode.PercentOutput, 0.0)
            }
            winchMoving = false
        }

        val pov = MainXbox.pov
        if (lastPov != pov && ElevatorSubsystem.closedLpControl) {
            when (pov) {
            // Right - Scale Height
                90 -> ClimbSubsystem.set(ControlMode.MotionMagic, ClimbConstants.SCALE_POS.toDouble())
            // Down - Original Position
                180 -> ClimbSubsystem.set(ControlMode.MotionMagic, ClimbConstants.CLIMB_POS.toDouble())
            }
        }
        lastPov = pov
    }
}