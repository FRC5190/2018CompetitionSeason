/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.auto.AutoHelper
import frc.team5190.robot.auto.StartingPositions
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.drive.Gear
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.NavX
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

    // Shows a drop down on dashboard that allows us to select which mode we want
    private val sideChooser = SendableChooser<StartingPositions>()

    // Shows a dropdown of the controllers that weill be used.
    private val controllerChooser = SendableChooser<String>()

    // Variable that stores which side of the switch to go to.
    private var switchSide = MatchData.OwnedSide.UNKNOWN

    // Variable that stores which side of the scale to go to.
    private var scaleSide = MatchData.OwnedSide.UNKNOWN

    // Variable that stores LS-LR / RS-RL profile setting
    private var lslr = SendableChooser<String>()

    /**
     * Executed when robot code first launches and is ready to be initialized.
     */
    override fun robotInit() {
        // https://www.chiefdelphi.com/forums/showthread.php?p=1724798
        LiveWindow.disableAllTelemetry()

        DriveSubsystem
//        VisionSubsystem
        IntakeSubsystem
        ElevatorSubsystem
        ArmSubsystem
        NavX

        StartingPositions.values().forEach { sideChooser.addObject(it.name.toLowerCase().capitalize(), it) }
        sideChooser.addDefault("Left", StartingPositions.LEFT)

        SmartDashboard.putData("Starting Position", sideChooser)

        lslr.addObject("2 Switch", "2 Switch")
        lslr.addObject("2 Scale", "2 Scale")
        lslr.addDefault("2 Scale", "2 Scale")

        SmartDashboard.putData("LS-LR / RS-RL Preference", lslr)

        controllerChooser.addObject("Xbox", "Xbox")
        controllerChooser.addObject("Bongo", "Bongo")
        controllerChooser.addDefault("Xbox", "Xbox")

        SmartDashboard.putData("Controller", controllerChooser)

//        CameraServer.getInstance().startAutomaticCapture(0)
    }

    /**
     * Executed periodically.
     */
    override fun robotPeriodic() {

        // Debug information
        SmartDashboard.putNumber("Left Motor RPM", Maths.nativeUnitsPer100MsToRPM(DriveSubsystem.falconDrive.leftMaster.getSelectedSensorVelocity(0)))
        SmartDashboard.putNumber("Right Motor RPM", Maths.nativeUnitsPer100MsToRPM(DriveSubsystem.falconDrive.rightMaster.getSelectedSensorVelocity(0)))

        SmartDashboard.putNumber("Left Encoder Position", DriveSubsystem.falconDrive.leftEncoderPosition.toDouble())
        SmartDashboard.putNumber("Right Encoder Position", DriveSubsystem.falconDrive.rightEncoderPosition.toDouble())

        SmartDashboard.putNumber("Left Encoder to Feet", Maths.nativeUnitsToFeet(DriveSubsystem.falconDrive.leftEncoderPosition))
        SmartDashboard.putNumber("Right Encoder to Feet", Maths.nativeUnitsToFeet(DriveSubsystem.falconDrive.rightEncoderPosition))

        SmartDashboard.putNumber("Elevator Encoder Position", ElevatorSubsystem.currentPosition.toDouble())

        SmartDashboard.putNumber("Arm Encoder Position", ArmSubsystem.currentPosition.toDouble())

        SmartDashboard.putNumber("Arm Motor Amperage", ArmSubsystem.amperage)
        SmartDashboard.putNumber("Elevator Motor Amperage", ElevatorSubsystem.amperage)

        SmartDashboard.putData("Elevator Subsystem", ElevatorSubsystem)
        SmartDashboard.putData("Drive Subsystem", DriveSubsystem)
        SmartDashboard.putData("Arm Subsystem", ArmSubsystem)
        SmartDashboard.putData("Intake Subsystem", IntakeSubsystem)

        SmartDashboard.putData("Gyro", NavX)

        SmartDashboard.putNumber("Angle", NavX.angle)

        Scheduler.getInstance().run()
    }

    /**
     * Executed when autonomous is initialized
     */
    override fun autonomousInit() {

        DriveSubsystem.autoReset()
        DriveSubsystem.falconDrive.gear = Gear.HIGH
        NavX.reset()

        this.pollForFMSData()
        AutoHelper.getAuto(StartingPositions.LEFT, switchSide, scaleSide, lslr.selected).start()
//        TurnCommand(12.3).start()
    }


    override fun autonomousPeriodic() {}

    /**
     * Executed once when robot is disabled.
     */
    override fun disabledInit() {}

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
