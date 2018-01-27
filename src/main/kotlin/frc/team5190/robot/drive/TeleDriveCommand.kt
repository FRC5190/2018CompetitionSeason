/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.*

class TeleDriveCommand : Command() {

    init {
        this.requires(DriveSubsystem)
    }

    /**
     * Called periodically until the command is finished or until interrupted.
     */
    override fun execute() {
        // TODO Configure Velocity Drive
        val mode = ControlMode.PercentOutput

        when (DriveSubsystem.controlMode) {
            DriveMode.ARCADE -> DriveSubsystem.falconDrive.arcadeDrive(-MainXbox.getLeftY(), MainXbox.getLeftX())
            DriveMode.CURVE -> DriveSubsystem.falconDrive.curvatureDrive(mode, -MainXbox.getLeftY(), MainXbox.getLeftX(), MainXbox.aButton)
            DriveMode.TANK -> {
                if (DriveSubsystem.controller == "Bongo") {
                    DriveSubsystem.falconDrive.tankDrive(mode, Bongos.getLeftBongoSpeed(), Bongos.getRightBongoSpeed())
                } else {
                    DriveSubsystem.falconDrive.tankDrive(mode, -MainXbox.getLeftY(), -MainXbox.getRightY())
                }
            }
        }
        DriveSubsystem.falconDrive.gear = when {
            MainXbox.getTriggerAxis(GenericHID.Hand.kRight) < 0.5 -> Gear.LOW
            else -> Gear.HIGH
        }
    }

    override fun isFinished() = false
}
