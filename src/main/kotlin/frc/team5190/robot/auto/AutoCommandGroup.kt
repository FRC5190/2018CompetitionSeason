package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.elevator.AutoElevatorCommand
import frc.team5190.robot.elevator.ElevatorPosition
import frc.team5190.robot.intake.*

class AutoCommandGroup(initialPath: Paths) : CommandGroup() {
    init {
        this.addSequential(object : CommandGroup() {
            init {
                this.addParallel(MotionProfileCommand(initialPath))
                this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
            }
        })

        this.addSequential(IntakeCommand(IntakeDirection.OUT, true, 0.5))

        this.addSequential(object : CommandGroup() {
            init {
                this.addParallel(IntakeHoldCommand(), 0.5)
                this.addParallel(MotionProfileCommand(Paths.LEFT_SWITCH_TO_CENTER, true))
            }
        })

        this.addSequential(TurnCommand(0.0))

        this.addSequential(object : CommandGroup() {
            init {
                this.addParallel(AutoElevatorCommand(ElevatorPosition.INTAKE))
                this.addParallel(AutoArmCommand(ArmPosition.DOWN))
                this.addParallel(MotionProfileCommand(Paths.STRAIGHT_INTO_PYRAMID))
                this.addParallel(IntakeCommand(IntakeDirection.IN, true, 3.0))
            }
        })
//
//        this.addSequential(IntakeHoldCommand())
////        this.addSequential(TurnCommand(-140.0))
////        this.addSequential(MotionProfileCommand(Paths.CENTER_TO_INTAKE))
////        this.addSequential(TurnCommand(180.0))
////        this.addSequential(IntakeCommand(IntakeDirection.OUT, true))
////        this.addSequential(IntakeCommand(IntakeDirection.OUT))
    }
}