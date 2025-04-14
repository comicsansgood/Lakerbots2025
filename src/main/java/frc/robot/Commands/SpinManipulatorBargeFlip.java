package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class SpinManipulatorBargeFlip extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  public Manipulator manipulator;
  public double setpoint;

  public SpinManipulatorBargeFlip(Manipulator manipulator, double setpoint) {
    this.manipulator = manipulator;
    this.setpoint = setpoint;
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute() {

    if(manipulator.ismanipulatorClearToFlip()){
    manipulator.manipulatorSpin(1);
    }
    else{
      manipulator.manipulatorSpin(-0.3);
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
  return false;
  }
}
