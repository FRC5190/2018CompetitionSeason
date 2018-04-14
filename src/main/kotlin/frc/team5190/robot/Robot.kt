/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.CommandGroup
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
import frc.team5190.robot.util.Maths
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


    private val sideChooser = SendableChooser<StartingPositions>()

    private val crossAutoChooser = SendableChooser<AutoModes>()
    private val sameSideAutoChooser = SendableChooser<AutoModes>()

    var switchSide = MatchData.OwnedSide.UNKNOWN
    var scaleSide = MatchData.OwnedSide.UNKNOWN

    var sideChooserSelected = StartingPositions.LEFT
    var sameSideAutoSelected = AutoModes.FULL
    var crossAutoSelected = AutoModes.FULL

    private var autonomousRoutine: CommandGroup? = null


    // Shows a dropdown of the controllers that will be used.
    private val controllerChooser = SendableChooser<String>()

    // Variable that stores if FMS data has been received
    var dataRec = false
        private set


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

        CameraServer.getInstance().startAutomaticCapture().apply {
            setResolution(640, 480)
            setFPS(20)
        }

        StartingPositions.values().forEach { sideChooser.addObject(it.name.toLowerCase().capitalize(), it) }

        AutoModes.values().forEach { sameSideAutoChooser.addObject(it.name.toLowerCase().capitalize() + " (${it.numCubes})", it) }
        AutoModes.values().forEach { crossAutoChooser.addObject(it.name.toLowerCase().capitalize() + " (${it.numCubes})", it) }


        SmartDashboard.putData("Starting Position", sideChooser)

        SmartDashboard.putData("Cross Scale Mode", crossAutoChooser)
        SmartDashboard.putData("Same Side Scale Mode", sameSideAutoChooser)

//        sameSideAutoSelected = sameSideAutoChooser.selected
//        crossAutoSelected = crossAutoChooser.selected
//        sideChooserSelected = sideChooser.selected
    }

    /**
     * Executed periodically.
     */
    override fun robotPeriodic() {
        SmartDashboard.putNumber("Pigeon Corrected Angle", Pigeon.correctedAngle)

        if (!INSTANCE!!.isOperatorControl && autonomousRoutine?.isRunning != true) {
            // Regenerate the routine if any variables have changed.
            if (sideChooser.selected != sideChooserSelected ||
                    sameSideAutoChooser.selected != sameSideAutoSelected ||
                    crossAutoChooser.selected != crossAutoSelected ||
                    MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) != switchSide ||
                    MatchData.getOwnedSide(MatchData.GameFeature.SCALE) != scaleSide) {

                sideChooserSelected = sideChooser.selected
                sameSideAutoSelected = sameSideAutoChooser.selected
                crossAutoSelected = crossAutoChooser.selected

                switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
                scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)

                dataRec = true

                autonomousRoutine = AutoHelper.getAuto(sideChooserSelected, switchSide, scaleSide, sameSideAutoSelected, crossAutoSelected)
            }
        }

        Scheduler.getInstance().run()
    }

    /**
     * Executed when autonomous is initialized
     */
    override fun autonomousInit() {

        DriveSubsystem.autoReset()
        IntakeSubsystem.enableVoltageCompensation()
        Pigeon.reset()

        Pigeon.angleOffset = if (sideChooser.selected == StartingPositions.CENTER) 0.0 else 180.0
    }

    override fun autonomousPeriodic() {
        if (autonomousRoutine?.isRunning == false) autonomousRoutine?.start()
    }

    /**
     * Executed once when robot is disabled.
     */
    override fun disabledInit() {
        IdleClimbCommand().start()
        ClimbSubsystem.climbState = false
    }


    /**
     * Executed when teleop is initialized
     */
    override fun teleopInit() {

        ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition.toDouble())
        ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())
        IntakeSubsystem.disableVoltageCompensation()

       autonomousRoutine?.cancel()

        DriveSubsystem.teleopReset()
        DriveSubsystem.controller = controllerChooser.selected ?: "Xbox"
    }
}
