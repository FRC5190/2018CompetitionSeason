package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.Robot
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.util.MotorIDs
import frc.team5190.robot.util.SolenoidIDs
import frc.team5190.robot.util.configCurrentLimiting

object IntakeSubsystem : Subsystem() {

    val intakeTalon = TalonSRX(MotorIDs.INTAKE_LEFT)
    val intakeSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.INTAKE)

    init {
        // hardware for this subsystem includes two motors in master-slave config and a solenoid
        intakeTalon.inverted = false

        val intakeTalonSlave = TalonSRX(MotorIDs.INTAKE_RIGHT)
        intakeTalonSlave.follow(intakeTalon)
        intakeTalonSlave.inverted = true

        // current limiting
        intakeTalon.configCurrentLimiting(20, 200, 10, 10)
        intakeTalonSlave.configCurrentLimiting(20, 200, 10, 10)

        // other configuration
        reset()
    }

    fun reset() {
        // nothing to reset for this subsystem
    }

    fun set(controlMode: ControlMode, motorOutput: Double) {
        // TODO: check whether the motor is in good state before setting the power.
        intakeTalon.set(controlMode, motorOutput)
    }

    val intakeMotorAmperage
        get() = intakeTalon.outputCurrent

    override fun initDefaultCommand() {
        defaultCommand = IntakeHoldCommand()
    }

    private var teleIntake = false

    override fun periodic() {
        // command orchestration
        if (!Robot.INSTANCE!!.isOperatorControl) {
            return
        }

        when {
            MainXbox.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5 -> {
                if (ElevatorSubsystem.nativeUnitsToInches(ElevatorSubsystem.currentPosition) >= 12 ||
                        ArmSubsystem.currentPosition >= ArmPosition.MIDDLE.ticks - 100) {
                    IntakeCommand(IntakeDirection.OUT).start()
                } else {
                    IntakeCommand(IntakeDirection.IN).start()
                }
                teleIntake = true
            }
            teleIntake -> {
                currentCommand?.cancel()
                teleIntake = false
            }
        }
    }
}

enum class IntakeDirection {
    IN, OUT
}