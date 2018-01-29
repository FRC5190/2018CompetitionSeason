package frc.team5190.robot.auto

import edu.wpi.first.wpilibj.command.CommandGroup
import frc.team5190.robot.arm.ArmPosition
import frc.team5190.robot.arm.AutoArmCommand
import frc.team5190.robot.elevator.AutoElevatorCommand
import frc.team5190.robot.elevator.ElevatorPosition
import frc.team5190.robot.intake.IntakeCommand
import frc.team5190.robot.intake.IntakeDirection

class AutoCommandGroup(initialPath: Paths) : CommandGroup() {
    init {
        this.addSequential(object : CommandGroup() {
            init{
                this.addParallel(MotionProfileCommand(initialPath))
                this.addParallel(AutoElevatorCommand(ElevatorPosition.SWITCH))
                this.addParallel(AutoArmCommand(ArmPosition.MIDDLE))
            }
        })
//        this.addSequential(TurnCommand(90.0))
        this.addSequential(IntakeCommand(IntakeDirection.OUT, true))
    }
}