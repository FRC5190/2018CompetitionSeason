/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.auto.AutoCommand
import frc.team5190.robot.auto.AutoHelper
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.Hardware

/**
 * Main robot class
 */
class Robot : IterativeRobot() {

    init {
        DriveSubsystem
        NavX
    }

    // Shows a drop down on dashboard that allows us to select which mode we want
    private val autoChooser = SendableChooser<AutoHelper>()

    /**
     * Executed when robot code first launches and is ready to be initialized.
     */
    override fun robotInit() {
        AutoHelper.values().forEach { autoChooser.addObject(it.name, it) }

        SmartDashboard.putData("Auto Mode", autoChooser)
    }

    /**
     * Executed periodically.
     */
    override fun robotPeriodic() {

        // Debug information
        SmartDashboard.putNumber("Left Motor RPM", DriveSubsystem.falconDrive.leftMaster.getSelectedSensorVelocity(0)
                * 600.0 / 1440.0)
        SmartDashboard.putNumber("Right Motor RPM", DriveSubsystem.falconDrive.rightMaster.getSelectedSensorVelocity(0)
                * 600.0 / 1440.0)

        SmartDashboard.putNumber("Left Encoder Position", DriveSubsystem.falconDrive.leftEncoderPosition.toDouble())
        SmartDashboard.putNumber("Right Encoder Position", DriveSubsystem.falconDrive.rightEncoderPosition.toDouble())

        SmartDashboard.putNumber("Left Encoder to Feet", ((DriveSubsystem.falconDrive.leftEncoderPosition.toDouble() / Hardware.NATIVE_UNITS_PER_ROTATION)
                * (2 * Math.PI * Hardware.WHEEL_RADIUS)) / 12)
        SmartDashboard.putNumber("Right Encoder to Feet", ((DriveSubsystem.falconDrive.rightEncoderPosition.toDouble() / Hardware.NATIVE_UNITS_PER_ROTATION)
                * (2 * Math.PI * Hardware.WHEEL_RADIUS)) / 12)

        SmartDashboard.putData("Gyro", NavX)

        Scheduler.getInstance().run()
    }

    /**
     * Executed when autonomous is initialized
     */
    override fun autonomousInit() {
        NavX.reset()
        AutoCommand(AutoHelper.TEST, true).start()
    }


    override fun disabledInit() {
        DriveSubsystem.reset()
    }

    /**
     * Executed when teleop is initialized
     */
    override fun teleopInit() {
        DriveSubsystem.currentCommand?.cancel()
    }
}
