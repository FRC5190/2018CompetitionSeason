/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.DriverStation
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
import openrio.powerup.MatchData

class Robot : IterativeRobot() {

    companion object {
        var INSTANCE: Robot? = null
    }

    init {
        INSTANCE = this
    }

    // Autonomous variables
    private val sideChooser = SendableChooser<StartingPositions>()

    private val crossAutoChooser = SendableChooser<AutoModes>()
    private val sameSideAutoChooser = SendableChooser<AutoModes>()

    var switchSide = MatchData.OwnedSide.UNKNOWN
    var scaleSide = MatchData.OwnedSide.UNKNOWN

    var sideChooserSelected = StartingPositions.LEFT
    var sameSideAutoSelected = AutoModes.FULL
    var crossAutoSelected = AutoModes.FULL

    private var autonomousRoutine: CommandGroup? = null
    private var hasRunAuto = false

    var fmsDataReceived = false


    override fun robotInit() {
        // https://www.chiefdelphi.com/forums/showthread.php?p=1724798
        LiveWindow.disableAllTelemetry()

        // Initialize subsystems and sensors
        DriveSubsystem
        IntakeSubsystem
        ElevatorSubsystem
        ClimbSubsystem
        ArmSubsystem

        Pathreader
        Localization
        NetworkInterface
        Canifier
        Lidar
        Pigeon
        LEDs

        // Autonomous modes on Dashboard
        StartingPositions.values().forEach { sideChooser.addObject(it.name.toLowerCase().capitalize(), it) }

        AutoModes.values().forEach {
            sameSideAutoChooser.addObject(it.name.toLowerCase().capitalize() + " (${it.numCubes})", it)
            crossAutoChooser.addObject(it.name.toLowerCase().capitalize() + " (${it.numCubes})", it)
        }

        SmartDashboard.putData("Starting Position", sideChooser)

        SmartDashboard.putData("Cross Scale Mode", crossAutoChooser)
        SmartDashboard.putData("Same Side Scale Mode", sameSideAutoChooser)

        // Reset subsystems for autonomous
        IntakeSubsystem.enableVoltageCompensation()
        DriveSubsystem.autoReset()

        Pigeon.reset()

        ClimbSubsystem.climbState = false
    }

    override fun robotPeriodic() {
        Pigeon.update()

        // Logging
        SmartDashboard.putNumber("Pigeon Corrected Angle", Pigeon.correctedAngle)


        // Receives game data from the FMS and generates autonomous routine
        if (!INSTANCE!!.isOperatorControl && autonomousRoutine?.isRunning != true && (!fmsDataReceived || INSTANCE!!.isDisabled)) {
            try {
                if (sideChooser.selected != sideChooserSelected ||
                        sameSideAutoChooser.selected != sameSideAutoSelected ||
                        crossAutoChooser.selected != crossAutoSelected ||
                        MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) != switchSide ||
                        MatchData.getOwnedSide(MatchData.GameFeature.SCALE) != scaleSide ||
                        hasRunAuto) {

                    DriveSubsystem.autoReset()

                    sideChooserSelected = sideChooser.selected
                    sameSideAutoSelected = sameSideAutoChooser.selected
                    crossAutoSelected = crossAutoChooser.selected

                    switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
                    scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)

                    fmsDataReceived = switchSide != MatchData.OwnedSide.UNKNOWN && scaleSide != MatchData.OwnedSide.UNKNOWN

                    println("Received Game Specific Data: ${DriverStation.getInstance().gameSpecificMessage}")

                    // Reset gyro
                    Pigeon.reset()
                    Pigeon.angleOffset = if (sideChooserSelected == StartingPositions.CENTER) 0.0 else 180.0

                    autonomousRoutine = AutoHelper.getAuto(sideChooserSelected, switchSide, scaleSide, sameSideAutoSelected, crossAutoSelected)
                }
            } catch (ignored: Exception) {
            }
        }
        Scheduler.getInstance().run()
    }

    override fun autonomousPeriodic() {
        // Runs the autonomous routine once data has been received
        if (autonomousRoutine?.isRunning == false && !hasRunAuto && switchSide != MatchData.OwnedSide.UNKNOWN && scaleSide != MatchData.OwnedSide.UNKNOWN) {
            autonomousRoutine?.start()
            hasRunAuto = true
        }
    }

    override fun disabledInit() {

        hasRunAuto = false

        // Clean up from climbing
        IdleClimbCommand().start()
        ClimbSubsystem.climbState = false
    }

    override fun teleopInit() {
        // Lock elevator in place
        ElevatorSubsystem.set(ControlMode.MotionMagic, ElevatorSubsystem.currentPosition.toDouble())
        ArmSubsystem.set(ControlMode.MotionMagic, ArmSubsystem.currentPosition.toDouble())
        IntakeSubsystem.disableVoltageCompensation()


        // Clean up from autonomous
        autonomousRoutine?.cancel()
        hasRunAuto = true

        DriveSubsystem.teleopReset()
        DriveSubsystem.controller = "Xbox"
    }
}
