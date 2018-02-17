/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.CommandGroup
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.auto.*
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.drive.Gear
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.Maths
import frc.team5190.robot.util.commandGroup
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

    // Shows a drop down on dashboard that allows us to select which mode we want
    private val strategyChooser = SendableChooser<AutoMode>()

    // Shows a dropdown of the controllers that weill be used.
    private val controllerChooser = SendableChooser<String>()

    // Variable that stores which side of the switch to go to.
    private var switchSide = MatchData.OwnedSide.UNKNOWN

    // Variable that stores which side of the scale to go to.
    private var scaleSide = MatchData.OwnedSide.UNKNOWN

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
        ArmSubsystem
        NavX

        StartingPositions.values().forEach { sideChooser.addObject(it.name.toLowerCase().capitalize(), it) }
        sideChooser.addDefault("Left", StartingPositions.LEFT)

        SmartDashboard.putData("Side Selector", sideChooser)

        AutoMode.values().forEach { strategyChooser.addObject(it.name.toUpperCase(), it) }
        strategyChooser.addDefault("TWO_CUBE_BOTH", AutoMode.TWO_CUBE_BOTH)

        controllerChooser.addObject("Xbox", "Xbox")
        controllerChooser.addObject("Bongo", "Bongo")
        controllerChooser.addDefault("Xbox", "Xbox")
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
        SmartDashboard.putNumber("Elevator Inches Position", ElevatorSubsystem.nativeUnitsToInches(ElevatorSubsystem.currentPosition))

        SmartDashboard.putNumber("Arm Encoder Position", ArmSubsystem.currentPosition.toDouble())

        SmartDashboard.putNumber("Left Motor Amperage", DriveSubsystem.leftMotorAmperage)
        SmartDashboard.putNumber("Right Motor Amerpage", DriveSubsystem.rightMotorAmperage)

        SmartDashboard.putNumber("Arm Motor Amperage", ArmSubsystem.motorAmps)

        SmartDashboard.putData("Elevator Subsystem", ElevatorSubsystem)
        SmartDashboard.putData("Drive Subsystem", DriveSubsystem)
        SmartDashboard.putData("Arm Subsystem", ArmSubsystem)
        SmartDashboard.putData("Intake Subsystem", IntakeSubsystem)
        SmartDashboard.putData("Gyro", NavX)

        Scheduler.getInstance().run()
    }

    /**
     * Executed when autonomous is initialized
     */
    override fun autonomousInit() {
//      ResetElevatorCommand().start()
        DriveSubsystem.autoReset()
        DriveSubsystem.falconDrive.gear = Gear.HIGH
        NavX.reset()

        this.pollForFMSData()
        AutoHelper.getAuto(StartingPositions.CENTER, switchSide, scaleSide, strategyChooser.selected).start()
    }

    /**
     * Executed once when robot is disabled.
     */
    override fun disabledInit() {
    }

    /**
     * Executed when teleop is initialized
     */
    override fun teleopInit() {
//        commandGroup {
//            if (ArmSubsystem.currentPosition < ArmPosition.DOWN.ticks)
//                addSequential(AutoArmCommand(ArmPosition.DOWN))
//            addSequential(ResetElevatorCommand())
//        }.start()

        DriveSubsystem.currentCommand?.cancel()

        DriveSubsystem.teleopReset()
        DriveSubsystem.controller = controllerChooser.selected ?: "Xbox"
    }

    private fun pollForFMSData() {
        switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
        scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)
    }
}
