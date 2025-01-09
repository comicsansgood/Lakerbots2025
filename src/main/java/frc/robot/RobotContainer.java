package frc.robot;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();



  public RobotContainer() {
    configureBindings();
  }

  
  private void configureBindings() {}


  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
