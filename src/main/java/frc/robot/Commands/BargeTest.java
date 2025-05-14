package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class BargeTest extends Command {
  
  public Manipulator manipulator;

  public BargeTest(Manipulator manipulator) {
    this.manipulator = manipulator;
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.setmanipulator(Constants.ManipulatorConstants.manipulatorFlipPrepare);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
      return manipulator.manipulatorAtPosition();
  }
}
