/**
 * FRC Team 5190
 * Programming Team
 */

package frc.team5190.robot.drive

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.MainXbox
import frc.team5190.robot.util.MotorIDs
import frc.team5190.robot.util.SolenoidIDs

object DriveSubsystem : Subsystem() {

    // Establishes the drive mode for our different drivers.
    var controlMode = DriveMode.TANK
        set(value) {
            println("Changing DriveMode from $field to $value")
            field = value
            SmartDashboard.putString("Drive Mode", value.name)
        }

    var controller = "Xbox"

    var compressor = Compressor(SolenoidIDs.PCM)

    init {
        println("Drive Initialized")
        compressor.start()
    }

    // Creates an instance of FalconDrive, our custom drive class
    val falconDrive = FalconDrive(listOf(MotorIDs.FRONT_LEFT, MotorIDs.REAR_LEFT).map { WPI_TalonSRX(it) },
            listOf(MotorIDs.FRONT_RIGHT, MotorIDs.REAR_RIGHT).map { WPI_TalonSRX(it) },
            Solenoid(SolenoidIDs.PCM, SolenoidIDs.DRIVE))

    val leftMotorAmperage
        get() = falconDrive.leftMaster.outputCurrent

    val rightMotorAmperage
        get() = falconDrive.rightMaster.outputCurrent

    /**
     * Initializes the default command for the subsystem
     */
    override fun initDefaultCommand() {
        this.defaultCommand = TeleDriveCommand()
    }

    /**
     * Executed periodically. Used for switching drive modes.
     */
    override fun periodic() {
        when {
            MainXbox.backButtonPressed -> DriveMode.TANK
            MainXbox.startButtonPressed -> DriveMode.CURVE
            else -> null
        }?.let { controlMode = it }
    }

    fun teleopReset() = falconDrive.teleopReset()

    fun autoReset() = falconDrive.autoReset()
}

/**
 * Used for storing the various drive modes.
 */
enum class DriveMode {
    ARCADE,
    TANK,
    CURVE
}
