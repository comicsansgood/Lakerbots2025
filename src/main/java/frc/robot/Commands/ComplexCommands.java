package frc.robot.Commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
        m_manipulator.spinUntilNotDetected(0.1);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(elevatorPosition);
        m_manipulator.manipulatorGoToPosition(manipulatorPosition);
      }); 
    }
    

    public static Command centerCoral(){


      return Commands.sequence(
      Commands.runOnce(()->{
        m_manipulator.spinUntilDetected();
        m_manipulator.spinUntilNotDetected(.05);
        m_manipulator.manipulatorSpin(-0.1);}), 
      Commands.waitSeconds(1), 
      Commands.runOnce(()->{m_manipulator.manipulatorSpin(0);}));

      /*return Commands.runOnce(()->{
        m_manipulator.spinUntilDetected();
        m_manipulator.spinUntilNotDetected(.05);
        m_manipulator.manipulatorSpin(-0.1);
      }).andThen(Commands.waitSeconds(1)).andThen(m_manipulator.manipulatorSpin(0));*/
      
    }
    
    public Command goHome(){
      return Commands.runOnce(() -> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome);
      });
    }

    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/hatchbotinlined/commands/Autos.java
    /*public Command goHomeSequence(){
      return Commands.sequence(
        new FunctionalCommand(
          m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel),
          m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome),
          m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome)
        )
      );
    }*/ 
    // change manipulatorGoToPosition to include an "is finsihed" using the boolean and then use in the format below in the link
    // https://github.com/comicsansgood/Lakerbots2024/blob/master/src/main/java/frc/robot/commands/FeederCommands/FeederCenter.java
    
  
    public Command collectCoral(){
      return Commands.runOnce(() -> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorCollect);
        m_manipulator.spinUntilDetected();
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome);

      });
    }

    public Command collectAlgaeL2(){
      return Commands.runOnce(() -> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL2);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgeaCollect);
        m_manipulator.spinUntilDetected();
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome);
      });
    }

    public Command collectAlgeaL3(){
      return Commands.runOnce(() -> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL3);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgeaCollect);
        m_manipulator.spinUntilDetected();
        //m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);   where does it go with algea in it?
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome);
      });
    }

    public Command algeaProcessor(){
      return Commands.runOnce(() -> {

      });
    }

    public Command elevatorTravel(double targetPos){
      return Commands.runOnce(()-> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(targetPos);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome);
      });
    }





    /*public Command exampleCommand( type   name){
      return Commands.runOnce(()->{

      }, );
    }*/


    
  


}
