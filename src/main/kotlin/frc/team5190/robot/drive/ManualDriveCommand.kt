/*
 * Copyright (c) 2018 FRC Team 5190
 * Ryan Segerstrom, Prateek Machiraju
 */

/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.*

class ManualDriveCommand : Command() {

    init {
        this.requires(DriveSubsystem)
    }

    /**tel
     * Called periodically until the command is triggerState or until interrupted.
     */
    override fun execute() {
        val mode = ControlMode.PercentOutput

        when {
            DriveSubsystem.controlMode == DriveMode.ARCADE -> DriveSubsystem.falconDrive.arcadeDrive(-MainXbox.getLeftY(), MainXbox.getLeftX())
            DriveSubsystem.controlMode == DriveMode.CURVE -> DriveSubsystem.falconDrive.curvatureDrive(mode, -MainXbox.getLeftY(), MainXbox.getLeftX(), MainXbox.xButton)
            DriveSubsystem.controlMode == DriveMode.TANK -> when {
                DriveSubsystem.controller == "Bongo" -> DriveSubsystem.falconDrive.tankDrive(mode, Bongos.getLeftBongoSpeed(), Bongos.getRightBongoSpeed())
                else -> DriveSubsystem.falconDrive.tankDrive(mode, -MainXbox.getLeftY(), -MainXbox.getRightY())
            }
        }
    }

    override fun isFinished() = false
}
