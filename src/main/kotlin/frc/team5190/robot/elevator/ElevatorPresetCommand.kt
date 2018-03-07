package frc.team5190.robot.elevator

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team5190.robot.arm.*
import frc.team5190.robot.util.commandGroup

class ElevatorPresetCommand(elevatorPosition: ElevatorPreset) : CommandGroup() {
    init {
        when (elevatorPosition) {
            ElevatorPreset.SWITCH -> {
                addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                addParallel(commandGroup {
                    addSequential(object : AutoElevatorCommand(ElevatorPosition.FIRST_STAGE) {
                        override fun isFinished() = ArmSubsystem.currentPosition < ArmPosition.UP.ticks + 100 || ElevatorSubsystem.currentPosition < ElevatorPosition.SWITCH.ticks
                    })
                    addSequential(AutoElevatorCommand(ElevatorPosition.SWITCH))
                })
            }
            ElevatorPreset.SCALE -> {
                addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                addParallel(AutoElevatorCommand(ElevatorPosition.SCALE_HIGH))
            }
            ElevatorPreset.BEHIND -> {
                addParallel(AutoElevatorCommand(ElevatorPosition.SCALE))
                addParallel(commandGroup {
                    addSequential(object : AutoArmCommand(ArmPosition.UP) {
                        override fun isFinished() = ElevatorSubsystem.currentPosition > ElevatorPosition.FIRST_STAGE.ticks
                    })
                    addSequential(AutoArmCommand(ArmPosition.BEHIND))
                })
            }
            ElevatorPreset.INTAKE -> {
                addParallel(AutoArmCommand(ArmPosition.DOWN))
                addParallel(commandGroup {
                    addSequential(object : AutoElevatorCommand(ElevatorPosition.FIRST_STAGE) {
                        override fun isFinished() = ArmSubsystem.currentPosition < ArmPosition.UP.ticks + 100
                    })
                    addSequential(AutoElevatorCommand(ElevatorPosition.INTAKE))
                })
            }
        }
    }
}

enum class ElevatorPreset {
    INTAKE,
    SWITCH,
    SCALE,
    BEHIND
}