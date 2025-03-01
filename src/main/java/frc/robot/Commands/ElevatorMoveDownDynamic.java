package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorMoveDownDynamic extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  public Elevator elevator;
  public double setpoint;

  public ElevatorMoveDownDynamic(Elevator elevator, double setpoint) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.elevatorDownDynamic(setpoint);
    
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
      return elevator.elevatorAtPosition();
  }
}
