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
import frc.team5190.robot.sensors.*
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

    // Shows a dropdown of the controllers that will be used.
    private val controllerChooser = SendableChooser<String>()

    // Shows a dropdown of which auto to use
    private val autoChooser = SendableChooser<String>()

    // Variable that stores which side of the switch to go to.
    private var switchSide = MatchData.OwnedSide.UNKNOWN

    // Variable that stores which side of the scale to go to.
    private var scaleSide = MatchData.OwnedSide.UNKNOWN

    // Gets the alliance that 5190 is part of when competing
    var alliance = DriverStation.Alliance.Invalid

    // Variable that stores if FMS data has been received
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
        Canifier
        Lidar
        Pigeon
        LEDs

        StartingPositions.values().forEach { sideChooser.addObject(it.name.toLowerCase().capitalize(), it) }
        sideChooser.addDefault("Left", StartingPositions.LEFT)

        autoChooser.addObject("Legacy", "Legacy")
        autoChooser.addDefault("Modern", "Modern")

        SmartDashboard.putData("Starting Position", sideChooser)
        SmartDashboard.putData("Auto Mode", autoChooser)
    }

    /**
     * Executed periodically.
     */
    override fun robotPeriodic() {

        SmartDashboard.putNumber("Left Encoder Position", DriveSubsystem.falconDrive.leftEncoderPosition.toDouble())
        SmartDashboard.putNumber("Right Encoder Position", DriveSubsystem.falconDrive.rightEncoderPosition.toDouble())

        SmartDashboard.putNumber("Elevator Encoder Position", ElevatorSubsystem.currentPosition.toDouble())
        SmartDashboard.putNumber("Arm Encoder Position", ArmSubsystem.currentPosition.toDouble())

        SmartDashboard.putNumber("Gyro", Pigeon.correctedAngle)

        SmartDashboard.putBoolean("Cube In", IntakeSubsystem.isCubeIn)
        SmartDashboard.putData(ArmSubsystem)

        Scheduler.getInstance().run()
    }

    /**
     * Executed when autonomous is initialized
     */
    override fun autonomousInit() {

        pollForFMSData()

        DriveSubsystem.autoReset()
        IntakeSubsystem.enableVoltageCompensation()
        Pigeon.reset()

        Pigeon.angleOffset = if (sideChooser.selected == StartingPositions.CENTER) 0.0 else 180.0

        /* TESTING
        commandGroup {
            addSequential(MotionProfileCommand("LS-LL", "Drop First Cube", robotReversed = true, pathMirrored = false))
            addSequential(commandGroup {
                addParallel(MotionProfileCommand("LS-LL", "Pickup Second Cube", pathMirrored = false))
                addParallel(IntakeCommand(IntakeDirection.IN), 3.0)
            })
            addSequential(IntakeHoldCommand(), 0.001)
            addSequential(MotionProfileCommand("LS-LL", "Pickup Second Cube", robotReversed = true, pathReversed = true, pathMirrored = false))
            addSequential(IntakeCommand(IntakeDirection.OUT, speed = 1.0), 0.5)

            addSequential(IntakeHoldCommand(), 0.001)

            addSequential(commandGroup {
                addParallel(MotionProfileCommand("LS-LL", "Pickup Third Cube", pathMirrored = false))
                addParallel(IntakeCommand(IntakeDirection.IN), 3.0)
            })
            addSequential(IntakeHoldCommand(), 0.001)
            addSequential(MotionProfileCommand("LS-LL", "Pickup Third Cube", robotReversed = true, pathReversed = true, pathMirrored = false))

        }.start()
        */

        if (autoChooser.selected == "Legacy") {
            AutoHelper.LegacyAuto.getAuto(sideChooser.selected, switchSide, scaleSide).start()
        } else {
            AutoHelper.ModernAuto.getAuto(sideChooser.selected, switchSide, scaleSide).start()
        }
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

        pollForFMSData()

        ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition.toDouble())
        ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())
        IntakeSubsystem.disableVoltageCompensation()

        DriveSubsystem.currentCommand?.cancel()

        DriveSubsystem.teleopReset()
        DriveSubsystem.controller = controllerChooser.selected ?: "Xbox"
    }


    private fun pollForFMSData() {
        switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
        scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)

        dataRec = true
        alliance = DriverStation.getInstance().alliance
    }
}
