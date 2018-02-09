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
import frc.team5190.robot.util.*

object IntakeSubsystem : Subsystem() {

    private val currentBuffer = CircularBuffer(50)

    private val intakeTalon = TalonSRX(MotorIDs.INTAKE_LEFT)
    val intakeSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.INTAKE)

    private val intakeMotorAmperage
        get() = intakeTalon.outputCurrent

    private var teleIntake = false

    init {
        intakeTalon.inverted = false

        val intakeTalonSlave = TalonSRX(MotorIDs.INTAKE_RIGHT)
        intakeTalonSlave.follow(intakeTalon)
        intakeTalonSlave.inverted = true

        currentBuffer.configureForTalon(20, 1000)
    }

    fun set(controlMode: ControlMode, motorOutput: Double) {
        intakeTalon.set(controlMode, motorOutput)
    }

    override fun initDefaultCommand() {
        this.defaultCommand = IntakeHoldCommand()
    }

    override fun periodic() {
        currentBuffer.add(intakeMotorAmperage)
        val state = intakeTalon.limitCurrent(currentBuffer)

        if (!Robot.INSTANCE!!.isOperatorControl) return

        when {
            MainXbox.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5 -> {
                if (ElevatorSubsystem.nativeUnitsToInches(ElevatorSubsystem.currentPosition) >= 12 || ArmSubsystem.currentPosition >= ArmPosition.MIDDLE.ticks - 100) {
                    IntakeCommand(IntakeDirection.OUT, state).start()
                } else {
                    IntakeCommand(IntakeDirection.IN, state).start()
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