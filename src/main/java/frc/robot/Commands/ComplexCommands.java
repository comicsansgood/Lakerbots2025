package frc.robot.Commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FlapHook;
import frc.robot.subsystems.Manipulator;

public class ComplexCommands {


    public static FlapHook flapHook = new FlapHook();
    
    public static Elevator m_elevator = new Elevator();
    public static Manipulator m_manipulator = new Manipulator();

    public static double manipulatorPosition, elevatorPosition;

    public ComplexCommands(){}

    

    public Command score(int level){
      switch(level){
        case 1:
          manipulatorPosition = Constants.ManipulatorConstants.manipulatorCoralL1;
          elevatorPosition = Constants.ElevatorConstants.elevatorCoralL1;
          break;
        case 2: 
          manipulatorPosition = Constants.ManipulatorConstants.manipulatorCoralL2;
          elevatorPosition = Constants.ElevatorConstants.elevatorCoralL2;
          break;
        case 3:
          manipulatorPosition = Constants.ManipulatorConstants.manipulatorCoralL3;
          elevatorPosition = Constants.ElevatorConstants.elevatorCoralL3;
          break;
        case 4:
          manipulatorPosition = Constants.ManipulatorConstants.manipulatorCoralL4;
          elevatorPosition = Constants.ElevatorConstants.elevatorCoralL4;
          break;
      }


      return Commands.runOnce(() -> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(elevatorPosition);
        m_manipulator.manipulatorGoToPosition(manipulatorPosition);
      }); 
    }

    public Command goHome(){
      return Commands.runOnce(() -> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome);
      });
    }

    public Command collectCoral(){
      return Commands.runOnce(() -> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCollect);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorCollect);
        m_manipulator.spinUntilDetected();
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome);

      });
    }

    /*public Command exampleCommand( type   name){
      return Commands.runOnce(()->{

      }, );
    }*/


    
  


}
