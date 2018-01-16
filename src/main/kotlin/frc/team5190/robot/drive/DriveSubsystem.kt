package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.drive.commands.TeleDriveCommand
import frc.team5190.robot.util.MotorIDs

object DriveSubsystem : Subsystem() {

    init {
        println("Drive Initialized")
    }

    var controlMode = DriveMode.TANK
        set(value) {
            println("Changing DriveMode from $field to $value")
            field = value
        }

    val falconDrive = FalconDrive(listOf(MotorIDs.FRONT_LEFT, MotorIDs.REAR_LEFT).map { WPI_TalonSRX(it) },
            listOf(MotorIDs.FRONT_RIGHT, MotorIDs.REAR_RIGHT).map { WPI_TalonSRX(it) })

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
