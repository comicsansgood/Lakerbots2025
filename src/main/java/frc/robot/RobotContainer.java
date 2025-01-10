package frc.robot;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();

  private XboxController driverXbox = new XboxController(0);
  private XboxController operatorXbox = new XboxController(1);


  public RobotContainer() {
    configureBindings();
  }

  
  private void configureBindings() {
    new JoystickButton(driverXbox, XboxController.Button.kA.value).onTrue(m_climber.climberGoToPosition(Constants.ClimberConstants.climberOut));
    new JoystickButton(driverXbox, XboxController.Button.kB.value).onTrue(m_climber.climberGoToPosition(Constants.ClimberConstants.climberIn));
    new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value).onTrue(m_manipulator.manipulatorGo(0.2));
    new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value).onTrue(m_manipulator.manipulatorGo(0));


  }


  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
