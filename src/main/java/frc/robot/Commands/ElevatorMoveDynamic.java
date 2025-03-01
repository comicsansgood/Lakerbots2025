package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorMoveDynamic extends Command {
  
  public Elevator elevator;
  public double setpoint;

  public ElevatorMoveDynamic(Elevator elevator, double setpoint) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    //-100       -50
    if(setpoint - elevator.elevatorLead.getPosition().getValueAsDouble() < 0){
      elevator.elevatorUpDynamic(setpoint);
    }else{
      elevator.elevatorDownDynamic(setpoint);
    }
    
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
