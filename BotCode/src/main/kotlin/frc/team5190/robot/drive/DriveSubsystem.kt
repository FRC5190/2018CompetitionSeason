package frc.team5190.robot.drive

import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.*
import frc.team5190.robot.drive.commands.TeleDriveCommand

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

    fun drive() {
        when (controlMode) {
            DriveMode.TANK -> falconDrive.tankDrive(MainXbox.leftY(), MainXbox.rightY())
            DriveMode.CURVE -> falconDrive.curvatureDrive(MainXbox.leftY(), MainXbox.leftX(), MainXbox.aButton)
        }
        when {
            MainXbox.backButtonPressed -> DriveMode.TANK
            MainXbox.startButtonPressed -> DriveMode.CURVE
            else -> null
        }?.let { controlMode = it }
    }
}

enum class DriveMode {
    TANK,
    CURVE
}
