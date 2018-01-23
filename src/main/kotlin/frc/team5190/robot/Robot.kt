/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.auto.AutoCommandGroup
import frc.team5190.robot.auto.AutoHelper
import frc.team5190.robot.auto.StartingPositions
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.Maths
import openrio.powerup.MatchData

/**
 * Main robot class
 */
class Robot : IterativeRobot() {

    init {
        DriveSubsystem
        ElevatorSubsystem
        NavX
    }

    // Shows a drop down on dashboard that allows us to select which mode we want
    private val sideChooser = SendableChooser<StartingPositions>()

    // Variable that stores which side of the switch to go to.
    private var switchSide = MatchData.OwnedSide.UNKNOWN

    // Variable that stores which side of the scale to go to.
    private var scaleSide = MatchData.OwnedSide.UNKNOWN

    /**
     * Executed when robot code first launches and is ready to be initialized.
     */
    override fun robotInit() {
        StartingPositions.values().forEach { sideChooser.addObject(it.name.toLowerCase().capitalize(), it) }
        SmartDashboard.putData("Starting Position", sideChooser)
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

        SmartDashboard.putData("Gyro", NavX)

        Scheduler.getInstance().run()
    }

    /**
     * Executed when autonomous is initialized
     */
    override fun autonomousInit() {
        ElevatorSubsystem.set(ControlMode.Position, ElevatorSubsystem.position)

        if (switchSide == MatchData.OwnedSide.UNKNOWN) switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
        if (scaleSide == MatchData.OwnedSide.UNKNOWN) scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)

        NavX.reset()
        AutoCommandGroup(AutoHelper.getPathFromData(sideChooser.selected, switchSide)).start()
    }


    override fun disabledInit() {
        if (switchSide == MatchData.OwnedSide.UNKNOWN) switchSide = MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)
        if (scaleSide == MatchData.OwnedSide.UNKNOWN) scaleSide = MatchData.getOwnedSide(MatchData.GameFeature.SCALE)

        DriveSubsystem.reset()
    }

    /**
     * Executed when teleop is initialized
     */
    override fun teleopInit() {
        ElevatorSubsystem.set(ControlMode.Position, ElevatorSubsystem.position)
        DriveSubsystem.currentCommand?.cancel()
    }
}
