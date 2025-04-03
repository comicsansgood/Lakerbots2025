package frc.robot.Commands;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber2;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FlapHook;
import frc.robot.subsystems.Manipulator;

public class ComplexCommands {


    public static FlapHook m_flapHook = RobotContainer.flapHook;
    public static Elevator m_elevator = RobotContainer.elevator;
    public static Manipulator m_manipulator = RobotContainer.manipulator;

    public static double manipulatorPosition, elevatorPosition;

    public static FunctionalCommands m_FunctionalCommands;

    public ComplexCommands(){}

  //---------this needs to be commented and tested----//
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
    
//--- INDEX the CORAL---///
    public static Command indexCoral(){//TODO:figure out interupt behavior with going home
      return Commands.sequence(
        m_manipulator.spinUntilDetected(0.15),
        m_manipulator.manipulatorSpinForTime(0.1, 0.35)//chagned time from .45 3/29/25 
        //m_manipulator.spinUntilNotDetected(0.1),
        //m_manipulator.manipulatorSpinForTime(-0.1, 0.5) /////Changed form 0.35 to 0.75
      );
    }

    

    //---Return to home with safe manipulator position----//
    public static Command scoreDynamic(int level){
      switch(level){
        case 1:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL1;break;
        case 2:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL2;break;
        case 3:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL3;break;
        case 4:elevatorPosition = Constants.ElevatorConstants.elevatorCoralL4;break;
      }
      return Commands.sequence(
        m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
        new ElevatorMoveDynamic(m_elevator, elevatorPosition)
      ); 
    }


    public static Command goToHomePose(){
      return Commands.sequence(
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),// this does not appear to be working correctly
      m_elevator.elevatorGoToPositionUntilThere(Constants.ElevatorConstants.elevatorHome),
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorHome));
    }
    

    public Command algeaCollect(){
      return Commands.sequence(
        m_manipulator.manipulatorSpinForTime(0.2, 0.2),
        m_manipulator.manipulatorSpinUntilCurrentReached(0.2, 0.1),
        m_manipulator.manipulatorSpin(0.05)//hold at 5 %
      );
    }

    //make algeacollect, make collect pos work, score in processor postion - elevator&anip

    public static Command collectAlgeaL2(){
      return Commands.sequence(
        m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL2),
        m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect),
        m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.3, -0.3)
      );
    }
    public static Command collectAlgeaL3(){
      return Commands.sequence(
        m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL3),
        m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect),
        m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.3, -0.3)
      );
    }

    public static Command goToProcessorPose(){
      return Commands.parallel(
        m_elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorProcess),
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgaeProcess)
      );
    }
    public Command elevatorTravel(double targetPos){
      return Commands.runOnce(()-> {
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel);
        m_elevator.elevatorGoToPosition(targetPos);
        m_manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome);
      });
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
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),// this does not appear to be working correctly
      new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorHome),
      m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorHome));
    }
    

    public static Command collectAlgeaL2Dynamic(){
      return Commands.sequence(
        m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
        new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorAlgaeL2).alongWith(m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect)),
        m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.3, -0.3)
      );
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
      return m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.3, -0.3);
    }

    public static Command collectAlgeaL3Dynamic(){
      return Commands.sequence(
        m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorTravel),
        new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorAlgaeL3).alongWith(m_manipulator.manipulatorGoToPositionUntilThere(Constants.ManipulatorConstants.manipulatorAlgeaCollect)),
        m_manipulator.manipulatorSpinUntilCurrentReachedWithWait(-0.3, -0.3)
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


    /* 
    public static Command elevatorGoUp(double setpoint){
  
      System.out.println("i am going insane");
      
      return Commands.runOnce(()->{
        m_elevator.elevatorUpDynamic(setpoint);
      });
    }
    public static Command elevatorGoDown(double setpoint){
      System.out.println("help");
      return Commands.runOnce(()->{
        m_elevator.elevatorGoDownDynamic(setpoint);
      });
    }*/

    public Command seqTest(){
      return Commands.sequence(
        goToHomePose(), 
        new ElevatorMoveDynamic(m_elevator, Constants.ElevatorConstants.elevatorAlgaeL2), 
        elevatorTravel(elevatorPosition
        ));
    }
  


}
