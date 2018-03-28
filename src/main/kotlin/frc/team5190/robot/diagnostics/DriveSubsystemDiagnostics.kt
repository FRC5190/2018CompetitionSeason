/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.diagnostics

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.TimedCommand
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.sensors.NavX
import kotlin.math.absoluteValue

class DriveSubsystemDiagnostics : TimedCommand(3.0) {

    val hasPassedTest
        get() = passedTest

    private var passedTest = false

    init {
        requires(DriveSubsystem)
    }

    override fun initialize() {
        NavX.reset()
        DriveSubsystem.resetEncoders()
    }

    override fun execute() {
        DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.3, 0.3)
    }

    override fun end() {
        if ((DriveSubsystem.falconDrive.leftEncoderPosition - DriveSubsystem.falconDrive.rightEncoderPosition).absoluteValue < 400 && NavX.angle.absoluteValue < 5.0
                && DriveSubsystem.falconDrive.allMasters.all { talons -> talons.getSelectedSensorPosition(0) > 100 }) {
            println("Drive Subsystem OK")
            passedTest = true
        } else {
            println("Drive Subsystem FAILED")
        }

    }
}