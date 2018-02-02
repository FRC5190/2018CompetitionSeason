package frc.team5190.robot.intake

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.command.Subsystem
import frc.team5190.robot.MainXbox
import frc.team5190.robot.Robot
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.elevator.ElevatorSubsystem
import frc.team5190.robot.util.MotorIDs
import frc.team5190.robot.util.SolenoidIDs

object IntakeSubsystem : Subsystem() {

    val intakeTalon = WPI_TalonSRX(MotorIDs.INTAKE_LEFT)


    val intakeSolenoid = Solenoid(SolenoidIDs.PCM, SolenoidIDs.INTAKE)

    init {
        val intakeTalonSlave = WPI_TalonSRX(MotorIDs.INTAKE_RIGHT)
        intakeTalonSlave.follow(intakeTalon)

        intakeTalonSlave.inverted = true
    }

    override fun initDefaultCommand() {
        defaultCommand = IntakeHoldCommand()
    }

    private var teleIntake = false

    override fun periodic() {
        if (Robot.INSTANCE!!.isOperatorControl) {
            if (MainXbox.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5) {
                if (ElevatorSubsystem.nativeUnitsToInches(ElevatorSubsystem.currentPosition) >= 12 || ArmSubsystem.currentPosition >= 450) {
                    IntakeCommand(IntakeDirection.OUT).start()
                } else {
                    IntakeCommand(IntakeDirection.IN).start()
                }
                teleIntake = true
            } else if (teleIntake) {
                currentCommand?.cancel()
                teleIntake = false
            }
        }
    }

}