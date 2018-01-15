package frc.team5190.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.drive.DriveTrain
import frc.team5190.robot.navigation.NAVCommand
import frc.team5190.robot.navigation.NAVHelper
import frc.team5190.robot.util.Hardware

class Robot : IterativeRobot() {

    lateinit var autoMode: NAVHelper
    var navCommand: NAVCommand = NAVCommand(NAVHelper.CENTERS_LEFT)

    var chooser = SendableChooser<String>()

    override fun robotInit() {

        chooser.addObject("LSL", "LSL")
        chooser.addObject("CSL", "CSL")
        chooser.addObject("RSL", "RSL")
        chooser.addObject("LSR", "LSR")
        chooser.addObject("CSR", "CSR")
        chooser.addObject("RSR", "RSR")

        chooser.addDefault("CSL", "CSL")

        SmartDashboard.putData("Auto Mode", chooser)

        DriveTrain
        navCommand.cancel()
    }

    override fun autonomousInit() {

        autoMode = when (chooser.selected) {
            "LSL" -> NAVHelper.LEFTS_LEFT
            "LSR" -> NAVHelper.LEFTS_RIGHT
            "CSL" -> NAVHelper.CENTERS_LEFT
            "CSR" -> NAVHelper.CENTERS_RIGHT
            "RSL" -> NAVHelper.RIGHTS_LEFT
            "RSR" -> NAVHelper.RIGHTS_RIGHT

            else -> NAVHelper.CENTERS_LEFT // TODO implement just cross auto line method
        }

        navCommand = NAVCommand(autoMode)
        navCommand.start()
    }

    override fun autonomousPeriodic() {
        this.printToShuffleboard()
        Scheduler.getInstance().run()
    }

    override fun teleopInit() {
        navCommand.cancel()

    }

    override fun teleopPeriodic() {
        this.printToShuffleboard()
        Scheduler.getInstance().run()
    }

    override fun disabledInit() {
        navCommand.cancel()
        DriveTrain.initialize()
    }

    private fun printToShuffleboard() {
        SmartDashboard.putNumber("Left Motor RPM", DriveTrain.frontLeft.getSelectedSensorVelocity(0) * 600.0 / 1440.0)
        SmartDashboard.putNumber("Right Motor RPM", DriveTrain.frontRight.getSelectedSensorVelocity(0) * 600.0 / 1440.0)

        SmartDashboard.putNumber("Left Encoder Position", DriveTrain.getLeftEncoderPosition().toDouble())
        SmartDashboard.putNumber("Right Encoder Position", DriveTrain.getRightEncoderPosition().toDouble())

        SmartDashboard.putNumber("Left Encoder to Feet", ((DriveTrain.getLeftEncoderPosition().toDouble() / Hardware.NATIVE_UNITS_PER_ROTATION) * (2 * Math.PI * Hardware.WHEEL_RADIUS)) / 12)
        SmartDashboard.putNumber("Right Encoder to Feet", ((DriveTrain.getRightEncoderPosition().toDouble() / Hardware.NATIVE_UNITS_PER_ROTATION) * (2 * Math.PI * Hardware.WHEEL_RADIUS)) / 12)

    }
}

