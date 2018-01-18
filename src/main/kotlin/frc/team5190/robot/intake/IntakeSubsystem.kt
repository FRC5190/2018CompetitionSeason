package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.util.MotorIDs

object IntakeSubsystem : Subsystem() {

    val intakeTalon = WPI_TalonSRX(MotorIDs.INTAKE_LEFT)

    init {
        val intakeTalonSlave = WPI_TalonSRX(MotorIDs.INTAKE_RIGHT)
        intakeTalonSlave.follow(intakeTalon)

        intakeTalonSlave.inverted = true
    }

    override fun initDefaultCommand() {
        defaultCommand = IntakeCommand(IntakeDirection.NOTHING)
    }

    override fun periodic() {
        when {
            // Intake direction will be determined by where the elevator currently is
            MainXbox.xButtonPressed -> IntakeCommand(IntakeDirection.IN).start()
            MainXbox.xButtonReleased -> currentCommand?.cancel()
        }
    }

}

enum class IntakeDirection {
    IN, OUT, NOTHING
}