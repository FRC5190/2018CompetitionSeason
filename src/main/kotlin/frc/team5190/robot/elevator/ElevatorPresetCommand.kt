package frc.team5190.robot.elevator

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.ArmSubsystem
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.util.commandGroup

class ElevatorPresetCommand(elevatorPosition: ElevatorPreset) : CommandGroup() {
    init {
        // Elevator presets used in teleop and autonomous
        
        // Intelligent self-crash avoidance to make sure arm is out of the way 
        // before bringing down elevator.

        when (elevatorPosition) {
            // Switch preset
            ElevatorPreset.SWITCH -> {
                addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                addParallel(commandGroup {
                    addSequential(object : AutoElevatorCommand(ElevatorPosition.FIRST_STAGE) {
                        override fun isFinished() = ArmSubsystem.currentPosition < ArmPosition.UP.ticks + 100 || ElevatorSubsystem.currentPosition < ElevatorPosition.SWITCH.ticks
                    })
                    addSequential(AutoElevatorCommand(ElevatorPosition.SWITCH))
                })
            }
            
            // Scale preset
            ElevatorPreset.SCALE -> {
                addParallel(AutoArmCommand(ArmPosition.MIDDLE))
                addParallel(AutoElevatorCommand(ElevatorPosition.SCALE_HIGH))
            }

            // Scale behind preset which uses Lidar to detect scale height
            ElevatorPreset.BEHIND, ElevatorPreset.BEHIND_LIDAR -> {
                addParallel(if (elevatorPosition == ElevatorPreset.BEHIND_LIDAR) LidarElevatorCommand()
                else AutoElevatorCommand(ElevatorPosition.SCALE))

                addParallel(commandGroup {
                    addSequential(object : AutoArmCommand(ArmPosition.UP) {
                        override fun isFinished() = ElevatorSubsystem.currentPosition > ElevatorPosition.FIRST_STAGE.ticks
                    })
                    addSequential(AutoArmCommand(ArmPosition.BEHIND))
                })
            }

            // Intake preset
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

// Stores elevator presets
enum class ElevatorPreset {
    INTAKE,
    SWITCH,
    SCALE,
    BEHIND,
    BEHIND_LIDAR
}