package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;


public class BargeScoreFlip extends Command {
  
  public Elevator elevator;
  public Manipulator manipulator;

  public double elevatorSetpoint;
  public double manipulatorSetpoint;

  public BargeScoreFlip(double elevatorSetpoint, double manipulaotrSetpoint, Elevator elevator, Manipulator manipulator) {
    this. elevator = elevator;
    this. manipulator = manipulator;
    this. elevatorSetpoint = elevatorSetpoint;
    this. manipulatorSetpoint = manipulaotrSetpoint;

    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.setmanipulator(Constants.ManipulatorConstants.manipulatorFlipPrepare);
    System.out.println("part 1");
  }

  @Override
  public void execute() {

    elevator.elevatorUpDynamic(Constants.ElevatorConstants.elevatorBarge);
    System.out.println("part 2");

    if(elevator.getElevatorPosition() < elevatorSetpoint){
      manipulator.setmanipulator(Constants.ManipulatorConstants.manipulatorBargeScore);
      System.out.println("part 3");
    }
    if(manipulator.getManipulatorPosition() < manipulatorSetpoint && elevator.getElevatorPosition() < elevatorSetpoint){
      manipulator.manipulatorSpinMethod(1);
      System.out.println("part 4");
    }

  }
  
  @Override
  public void end(boolean interrupted) {
    manipulator.manipulatorSpinMethod(0);
  }

  @Override
  public boolean isFinished() {
  return manipulator.manipulatorAtSpecificPosition(Constants.ManipulatorConstants.manipulatorBargeScore);
  }
}
