/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
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
            DriveMode.TANK -> DriveSubsystem.falconDrive.tankDrive(mode, -MainXbox.getLeftY(), -MainXbox.getRightY(), false)
            DriveMode.CURVE -> DriveSubsystem.falconDrive.curvatureDrive(mode, -MainXbox.getLeftY(), MainXbox.getLeftX(), MainXbox.aButton)
        }
    }

    override fun isFinished() = false
}
