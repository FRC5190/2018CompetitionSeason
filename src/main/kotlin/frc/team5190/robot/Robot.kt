package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.navigation.NAVCommand
import frc.team5190.robot.navigation.NAVHelper
import frc.team5190.robot.sensors.NavX
import frc.team5190.robot.util.Hardware

class Robot : IterativeRobot() {

    init {
        DriveSubsystem
        NavX
    }

    // Shows a drop down on dashboard that allows us to select which mode we want
    private val autoChooser = SendableChooser<NAVHelper>()

    override fun robotInit() {
        NAVHelper.values().forEach { autoChooser.addObject(it.name, it) }
        SmartDashboard.putData("Auto Mode", autoChooser)
    }

    override fun autonomousInit() {
        NAVCommand(autoChooser.selected ?: NAVHelper.CENTERS_LEFT).start()
    }

    override fun teleopInit() {
        DriveSubsystem.currentCommand?.cancel()
    }

    override fun robotPeriodic() {
        Scheduler.getInstance().run()

        // Some debug information
        SmartDashboard.putNumber("Left Motor RPM", DriveSubsystem.falconDrive.leftMaster.getSelectedSensorVelocity(0) * 600.0 / 1440.0)
        SmartDashboard.putNumber("Right Motor RPM", DriveSubsystem.falconDrive.rightMaster.getSelectedSensorVelocity(0) * 600.0 / 1440.0)

        SmartDashboard.putNumber("Left Encoder Position", DriveSubsystem.falconDrive.leftEncoderPosition.toDouble())
        SmartDashboard.putNumber("Right Encoder Position", DriveSubsystem.falconDrive.rightEncoderPosition.toDouble())

        SmartDashboard.putNumber("Left Encoder to Feet", ((DriveSubsystem.falconDrive.leftEncoderPosition.toDouble() / Hardware.NATIVE_UNITS_PER_ROTATION) * (2 * Math.PI * Hardware.WHEEL_RADIUS)) / 12)
        SmartDashboard.putNumber("Right Encoder to Feet", ((DriveSubsystem.falconDrive.rightEncoderPosition.toDouble() / Hardware.NATIVE_UNITS_PER_ROTATION) * (2 * Math.PI * Hardware.WHEEL_RADIUS)) / 12)
    }
}
