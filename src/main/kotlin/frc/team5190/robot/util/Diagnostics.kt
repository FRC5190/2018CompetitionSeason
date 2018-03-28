/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

package frc.team5190.robot.util

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.climb.ClimbSubsystem
import frc.team5190.robot.drive.DriveSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.intake.IntakeSubsystem
import frc.team5190.robot.sensors.NavX
import kotlin.math.absoluteValue

class Diagnostics : Command() {

    private var diagnosticFinished = false

    private var driveTestFinished = false
    private var driveTestStartTime = -1L
    private var driveTestSuccess = false


    private var intakeTestFinished = false
    private var intakeTestStartTime = -1L
    private var intakeTestSuccess = false


    private var armTestFinished = false
    private var elevatorTestFinished = false

    init {
        requires(ArmSubsystem)
        requires(ClimbSubsystem)
        requires(DriveSubsystem)
        requires(ElevatorSubsystem)
        requires(IntakeSubsystem)
    }

    override fun execute() {
        if (!driveTestFinished) {
            if (driveTestStartTime == -1L) {
                driveTestStartTime = System.currentTimeMillis()
                NavX.reset()
                DriveSubsystem.autoReset()
            }
            DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.3, 0.3)

            if (System.currentTimeMillis() - driveTestStartTime > 3000) {
                driveTestSuccess = (DriveSubsystem.falconDrive.leftEncoderPosition - DriveSubsystem.falconDrive.rightEncoderPosition).absoluteValue < 400 && NavX.angle.absoluteValue < 5.0
                        && DriveSubsystem.falconDrive.allMasters.all { talons -> talons.getSelectedSensorPosition(0) > 0 }
                driveTestFinished = true
            }
        } else {
            DriveSubsystem.falconDrive.tankDrive(ControlMode.PercentOutput, 0.0, 0.0)

            if (!intakeTestFinished) {

            }

        }
    }


    override fun isFinished() = diagnosticFinished

}