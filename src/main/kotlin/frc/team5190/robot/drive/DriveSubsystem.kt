package frc.team5190.robot.drive

import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.MotorIds
import frc.team5190.robot.drive.commands.TeleDriveCommand
import frc.team5190.robot.talonListOf

object DriveSubsystem : Subsystem() {

    init {
        println("Drive Initialized")
    }

    var controlMode = DriveMode.TANK
        set(value) {
            println("Changing DriveMode from $field to $value")
            field = value
        }

    val falconDrive = FalconDrive(talonListOf(MotorIds.FRONT_LEFT_VAL, MotorIds.REAR_LEFT_VAL),
            talonListOf(MotorIds.FRONT_RIGHT_VAL, MotorIds.REAR_RIGHT_VAL))

    override fun initDefaultCommand() {
        this.defaultCommand = TeleDriveCommand()
    }

    override fun periodic() {
        when {
            MainXbox.backButtonPressed -> DriveMode.TANK
            MainXbox.startButtonPressed -> DriveMode.ARCADE
            else -> null
        }?.let { controlMode = it }
    }
}

enum class DriveMode {
    ARCADE,
    TANK,
    CURVE
}
