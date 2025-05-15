//Bookmark-7
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
    //If the diff between the setpoint and the pos is <0, move with the up profile 
    //(as elevator pos starts at 0 and goes nrgative as it moves up)
    //else use the down profile
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
