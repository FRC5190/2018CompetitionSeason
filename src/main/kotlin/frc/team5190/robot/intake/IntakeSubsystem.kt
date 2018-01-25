package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.util.MotorIDs
import frc.team5190.robot.util.SolenoidIDs

object IntakeSubsystem : Subsystem() {

    val intakeTalon = WPI_TalonSRX(MotorIDs.INTAKE_LEFT)


    val intakeSolenoid = Solenoid(SolenoidIDs.INTAKE)

    init {
        val intakeTalonSlave = WPI_TalonSRX(MotorIDs.INTAKE_RIGHT)
        intakeTalonSlave.follow(intakeTalon)

        intakeTalonSlave.inverted = true
    }

    override fun initDefaultCommand() {
        defaultCommand = IntakeHoldCommand()
    }

    override fun periodic() {
        when {
            // Intake direction will be determined by where the elevator currently is
            MainXbox.xButtonPressed -> {
                if (ElevatorSubsystem.nativeUnitsToInches(ElevatorSubsystem.currentPosition) < 12) {
                    IntakeCommand(IntakeDirection.IN).start()
                } else IntakeCommand(IntakeDirection.OUT).start()
            }
            MainXbox.xButtonReleased -> currentCommand?.cancel()
        }
    }

}