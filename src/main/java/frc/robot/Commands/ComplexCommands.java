package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FlapHook;
import frc.robot.subsystems.Manipulator;

public class ComplexCommands {


  //pull subsystems from robot container to avoid making new instance
  //Bookmark-3d
  public static FlapHook m_flapHook = RobotContainer.flapHook;
  public static Elevator m_elevator = RobotContainer.elevator;
  public static Manipulator m_manipulator = RobotContainer.manipulator;

  public static double manipulatorPosition, elevatorPosition;

  public static FunctionalCommands m_FunctionalCommands;

  public ComplexCommands(){}

  //depreciated
  public static Command score(int level){
    switch(level){
      case 1:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL1;break;
      case 2:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL2;break;
      case 3:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL3;break;
      case 4:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL4;break;
    }
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
      m_elevator.elevatorGoToPosition(elevatorPosition)
    ); 
  }
  



  public static Command indexCoral(){
    return Commands.sequence(
      m_manipulator.spinUntilDetected(0.3),
      m_manipulator.manipulatorSpinForTime(0.1, 0.35)
    );
  }

  

  public static Command scoreDynamic(int level){
    //case for each level that sets target pos
    //including an auxiliary "level 5" for our auto L4 scoring
    switch(level){
      case 1:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL1;break;
      case 2:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL2;break;
      case 3:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL3;break;
      case 4:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL4;break;
      case 5:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL4___AUTO;break;//for auto
    }
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
      new ElevatorMoveDynamic(m_elevator, elevatorPosition)
    ); 
  }


  //depreciated
  public static Command goToHomePose(){
    return Commands.sequence(
    m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
    m_elevator.elevatorGoToPositionUntilThere(Constants.ElevatorConstants.elevatorHome),
    m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorHome));
  }
  
  //depreciated
  public Command algeaCollect(){
    return Commands.sequence(
      m_manipulator.manipulatorSpinForTime(0.2, 0.2),
      m_manipulator.manipulatorSpinUntilCurrentReached(0.2, 0.1),
      m_manipulator.manipulatorSpin(0.05)//hold at 5 %
    );
  }

  //depreciated
  public static Command collectAlgeaL2(){
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
      m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL2),
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect),
      m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.3, -0.3)
    );
  }

  //depreciated
  public static Command collectAlgeaL3(){
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
      m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL3),
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect),
      m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.3, -0.3)
    );
  }

  //depreciated
  public static Command goToProcessorPose(){
    return Commands.parallel(
      m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorProcess),
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgaeProcess)
    );
  }

  //depreciated
  public Command elevatorTravel(double targetPos){
    return Commands.runOnce(()-> {
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
      m_elevator.elevatorGoToPosition(targetPos);
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome);
    });
  }


//TODO:finish flip sequence
  
  /*public static Command FlipAlgea(){
    return Commands.parallel(
      new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorBarge),
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorBargeScore),
      new SpinManipulatorBargeFlip(m_manipulator, elevatorPosition)
    );
  }*/
    

  public static Command PrepareAndFlip(){
    return Commands.sequence(
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgeaCollect)
      // andThen(FlipAlgea())
      );
  }

  
  public static Command algeaStore(){
    return Commands.parallel(
      m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL2),
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgaeTuck)
    );
  }
  
  
  public static Command bargeScore(){
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgaeTuck),
      m_elevator.elevatorGoToPositionUntilThere(Constants.ElevatorConstants.elevatorBarge),
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorBargeScore)
    );
  }






  public static Command goToHomePoseDynamic(){
    return Commands.sequence(
      //Bookmark-3b
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),// this does not appear to be working correctly
      new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorHome),
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorHome));
  }
  
  public static Command auto_goToAlgeaL2Pose(){//added 3/6/25 for algea in auto
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
      new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorAlgaeL2).alongWith(m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect))
    );
  }

  public static Command auto_goToAlgeaL3Pose(){
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
      new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorAlgaeL3).alongWith(m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect))
    );
  }

  public static Command auto_algeaCollect(){//added 3/6/25 for algea in auto
    return m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.4, -0.3);
  }

  public static Command collectAlgeaL2Dynamic(){
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
      new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorAlgaeL2).alongWith(m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect)),
      m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.4, -0.3)
    );
  }

  public static Command collectAlgeaL3Dynamic(){
    return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
      new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorAlgaeL3).alongWith(m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect)),
      m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.4, -0.3)
    );
  }

  public static Command goToProcessorPoseDynamic(){
    return Commands.parallel(
      new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorProcess),
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgaeProcess)
    );
  }

  public Command elevatorTravelDynamic(double targetPos){
    return Commands.runOnce(()-> {
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
      new ElevatorMoveDynamic(m_elevator, targetPos);
      m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome);
    });
  }
}
