/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.command.Command
import frc.team5190.robot.*
import frc.team5190.robot.elevator.ElevatorSubsystem

class TeleDriveCommand : Command() {

    init {
        this.requires(DriveSubsystem)
    }

    /**tel
     * Called periodically until the command is triggerState or until interrupted.
     */
    override fun execute() {
        // TODO Configure Velocity Drive
        val mode = ControlMode.Velocity

        when (DriveSubsystem.controlMode) {
            DriveMode.ARCADE -> DriveSubsystem.falconDrive.arcadeDrive(-MainXbox.getLeftY(), MainXbox.getLeftX())
            DriveMode.CURVE -> DriveSubsystem.falconDrive.curvatureDrive(mode, -MainXbox.getLeftY(), MainXbox.getLeftX(), MainXbox.xButton)
            DriveMode.TANK -> {
                if (DriveSubsystem.controller == "Bongo") {
                    DriveSubsystem.falconDrive.tankDrive(mode, Bongos.getLeftBongoSpeed(), Bongos.getRightBongoSpeed())
                } else {
                    DriveSubsystem.falconDrive.tankDrive(mode, -MainXbox.getLeftY(), -MainXbox.getRightY())
                }
            }
        }

        DriveSubsystem.falconDrive.gear = when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) || ElevatorSubsystem.nativeUnitsToInches(ElevatorSubsystem.currentPosition) > 60 -> Gear.LOW
            else -> Gear.HIGH
        }
    }

    override fun isFinished() = false
}
