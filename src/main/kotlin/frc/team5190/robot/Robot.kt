/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.auto.*
import frc.team5190.robot.climb.ClimbSubsystem
import frc.team5190.robot.climb.IdleClimbCommand
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.vision.Vision
import openrio.powerup.MatchData

/**
 * Main robot class
 */
class Robot : IterativeRobot() {

    companion object {
        var INSTANCE: Robot? = null
    }

    init {
        INSTANCE = this
    }

    // Shows a drop down on dashboard that allows us to select which mode we want
    private val sideChooser = SendableChooser<StartingPositions>()

    // Shows a dropdown of the controllers that weill be used.
    private val controllerChooser = SendableChooser<String>()

    // Variable that stores which side of the switch to go to.
    private var switchSide = MatchData.OwnedSide.UNKNOWN

    // Variable that stores which side of the scale to go to.
    private var scaleSide = MatchData.OwnedSide.UNKNOWN

    var alliance = DriverStation.Alliance.Invalid
    var dataRec = false


    /**
     * Executed when robot code first launches and is ready to be initialized.
     */
    override fun robotInit() {
        // https://www.chiefdelphi.com/forums/showthread.php?p=1724798
        LiveWindow.disableAllTelemetry()

        DriveSubsystem
        IntakeSubsystem
        ElevatorSubsystem
        ClimbSubsystem
        ArmSubsystem

        Pathreader
        NavX
        Vision

        StartingPositions.values().forEach { sideChooser.addObject(it.name.toLowerCase().capitalize(), it) }
        sideChooser.addDefault("Left", StartingPositions.LEFT)

        SmartDashboard.putData("Starting Position", sideChooser)
    }

    /**
     * Executed periodically.
     */
    override fun robotPeriodic() {

        SmartDashboard.putNumber("Left Encoder Position", DriveSubsystem.falconDrive.leftEncoderPosition.toDouble())
        SmartDashboard.putNumber("Right Encoder Position", DriveSubsystem.falconDrive.rightEncoderPosition.toDouble())

        SmartDashboard.putNumber("Elevator Encoder Position", ElevatorSubsystem.currentPosition.toDouble())
        SmartDashboard.putNumber("Arm Encoder Position", ArmSubsystem.currentPosition.toDouble())

        SmartDashboard.putNumber("Gyro", NavX.angle)

        SmartDashboard.putBoolean("Cube In", IntakeSubsystem.isCubeIn)

        Scheduler.getInstance().run()
    }

    /**
     * Executed when autonomous is initialized
     */
    override fun autonomousInit() {

        pollForFMSData()

        DriveSubsystem.autoReset()
        NavX.reset()

        AutoHelper.getAuto(sideChooser.selected, switchSide, scaleSide).start()
//        StraightDriveCommand(-8.0).start()
    }


    override fun autonomousPeriodic() {}

    /**
     * Executed once when robot is disabled.
     */
    override fun disabledInit() {
        IdleClimbCommand().start()
        ClimbSubsystem.climbState = false
    }

    override fun disabledPeriodic() {}

    /**
     * Executed when teleop is initialized
     */
    override fun teleopInit() {

        ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition.toDouble())
        ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())

        DriveSubsystem.currentCommand?.cancel()

        DriveSubsystem.teleopReset()
        DriveSubsystem.controller = controllerChooser.selected ?: "Xbox"
    }

    override fun teleopPeriodic() {}

    private fun pollForFMSData() {
        switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
        scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)

        dataRec = true
        alliance = DriverStation.getInstance().alliance
    }
}
