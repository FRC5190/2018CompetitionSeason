package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team5190.robot.MainXbox
import frc.team5190.robot.Robot
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.util.*

object IntakeSubsystem : Subsystem() {

    private val intakeTalon = TalonSRX(MotorIDs.INTAKE_LEFT)
    private val currentBuffer = CircularBuffer(25)

    val outputCurrent
        get() = currentBuffer.average

    val intakeSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.INTAKE)

    private var teleIntake = false

    init {
        intakeTalon.inverted = false

        val intakeTalonSlave = TalonSRX(MotorIDs.INTAKE_RIGHT)
        intakeTalonSlave.follow(intakeTalon)
        intakeTalonSlave.inverted = true
    }

    fun set(controlMode: ControlMode, motorOutput: Double) {
        intakeTalon.set(controlMode, motorOutput)
    }

    override fun initDefaultCommand() {
        this.defaultCommand = IntakeHoldCommand()
    }

    override fun periodic() {

        currentBuffer.add(intakeTalon.outputCurrent)

        if (!Robot.INSTANCE!!.isOperatorControl) return

        SmartDashboard.putNumber("Intake Motor Amps", this.outputCurrent)

        when {
            MainXbox.getBumper(GenericHID.Hand.kLeft) -> {
                IntakeCommand(IntakeDirection.IN).start()
                teleIntake = true
            }
            MainXbox.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5 -> {
                IntakeCommand(IntakeDirection.OUT).start()
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