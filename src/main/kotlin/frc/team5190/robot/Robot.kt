/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.auto.AutoHelper
import frc.team5190.robot.auto.StartingPositions
import frc.team5190.robot.climb.ClimbSubsystem
import frc.team5190.robot.climb.IdleClimbCommand
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.pathreader.Pathreader
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.Maths
import frc.team5190.robot.vision.VisionSubsystem
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

    // Variable that stores LS-LL / RS-RR profile setting
    private var lsll = SendableChooser<String>()

    // Variable that stores LS-LR / RS-RL profile setting
    private var lslr = SendableChooser<String>()

    // Variable that stores LS-RL / RS-LR profile setting
    private var lsrl = SendableChooser<String>()

    // Variable that stores LS-RR / RS-LL profile setting
    private var lsrr = SendableChooser<String>()


    /**
     * Executed when robot code first launches and is ready to be initialized.
     */
    override fun robotInit() {
        // https://www.chiefdelphi.com/forums/showthread.php?p=1724798
        LiveWindow.disableAllTelemetry()

        DriveSubsystem
        VisionSubsystem
        IntakeSubsystem
        ElevatorSubsystem
        ClimbSubsystem
        ArmSubsystem

        Pathreader

        NavX

        StartingPositions.values().forEach { sideChooser.addObject(it.name.toLowerCase().capitalize(), it) }
        sideChooser.addDefault("Left", StartingPositions.LEFT)

        lsll.addDefault("Mixed", "Mixed")
        lsll.addObject("2 Scale", "2 Scale")
        lsll.addObject("Straight", "Straight")

        lslr.addDefault("1 Switch", "1 Switch")
        lslr.addObject("2 Scale", "2 Scale")

        lsrl.addDefault("2 Scale", "2 Scale")

        lsrr.addDefault("Mixed", "Mixed")

        controllerChooser.addDefault("Xbox", "Xbox")
        controllerChooser.addObject("Bongo", "Bongo")


        SmartDashboard.putData("Controller", controllerChooser)

        SmartDashboard.putData("LS-LL / RS-RR", lsll)
        SmartDashboard.putData("LS-LR / RS-RL", lslr)
        SmartDashboard.putData("LS-RL / RS-LR", lsrl)
        SmartDashboard.putData("LS-RR / RS-LL", lsrr)

        SmartDashboard.putData("Starting Position", sideChooser)

        CameraServer.getInstance().startAutomaticCapture(0).apply {
            setResolution(100, 100)
            setFPS(15)
        }
    }

    /**
     * Executed periodically.
     */
    override fun robotPeriodic() {
//
//        SmartDashboard.putNumber("Left Motor RPM", Maths.nativeUnitsPer100MsToRPM(DriveSubsystem.falconDrive.leftMaster.getSelectedSensorVelocity(0)))
//        SmartDashboard.putNumber("Right Motor RPM", Maths.nativeUnitsPer100MsToRPM(DriveSubsystem.falconDrive.rightMaster.getSelectedSensorVelocity(0)))

        SmartDashboard.putNumber("Left Encoder Position", DriveSubsystem.falconDrive.leftEncoderPosition.toDouble())
        SmartDashboard.putNumber("Right Encoder Position", DriveSubsystem.falconDrive.rightEncoderPosition.toDouble())
//
        SmartDashboard.putNumber("Left Encoder to Feet", Maths.nativeUnitsToFeet(DriveSubsystem.falconDrive.leftEncoderPosition))
        SmartDashboard.putNumber("Right Encoder to Feet", Maths.nativeUnitsToFeet(DriveSubsystem.falconDrive.rightEncoderPosition))
//
//        SmartDashboard.putNumber("Elevator Encoder Position", ElevatorSubsystem.currentPosition.toDouble())
//
        SmartDashboard.putNumber("Arm Encoder Position", ArmSubsystem.currentPosition.toDouble())
//
//        SmartDashboard.putNumber("Arm Motor Amperage", ArmSubsystem.amperage)
//        SmartDashboard.putNumber("Elevator Motor Amperage", ElevatorSubsystem.amperage)
//
//        SmartDashboard.putData("Elevator Subsystem", ElevatorSubsystem)
//        SmartDashboard.putData("Drive Subsystem", DriveSubsystem)
//        SmartDashboard.putData("Arm Subsystem", ArmSubsystem)
//        SmartDashboard.putData("Intake Subsystem", IntakeSubsystem)


        SmartDashboard.putData("Gyro", NavX)
        SmartDashboard.putNumber("Pitch", NavX.pitch.toDouble())
        SmartDashboard.putNumber("Roll", NavX.roll.toDouble())

        Scheduler.getInstance().run()
    }

    /**
     * Executed when autonomous is initialized
     */
    override fun autonomousInit() {

        pollForFMSData()

        DriveSubsystem.autoReset()
        NavX.reset()

        AutoHelper.getAuto(sideChooser.selected, switchSide, scaleSide, arrayOf(lsll.selected, lslr.selected, lsrl.selected, lsrr.selected)).start()
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
    }
}
