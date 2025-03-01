package frc.robot.subsystems;


import javax.net.SocketFactory;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber2 extends SubsystemBase {

  public SparkMax climber2Motor;
  public SparkLimitSwitch limitSwitch;
  public SparkClosedLoopController closedLoopController;
  public SparkMaxConfig motorConfig;
  public double targetPos;
  private RelativeEncoder encoder; ///CREATE AN ENCODER TODO

  public double tolerance = 0.1;//TODO: tune this value


  public Climber2() {

    climber2Motor = new SparkMax(5, MotorType.kBrushless);

    limitSwitch = climber2Motor.getReverseLimitSwitch();

    closedLoopController = climber2Motor.getClosedLoopController();

    encoder = climber2Motor.getEncoder();//TODO

    motorConfig = new SparkMaxConfig();

    

    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    motorConfig.idleMode(IdleMode.kBrake);//TODO

    motorConfig.smartCurrentLimit(40);


    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.08)   // sounded nasty at 10.0  2/15
        .i(0)
        .d(0.1)//TODO:these pid values are trash ngl
        .outputRange(-1, 1);
        // Set PID values for velocity control in slot 1
       // .p(0.0001, ClosedLoopSlot.kSlot1)
        //.i(0, ClosedLoopSlot.kSlot1)
        //.d(0, ClosedLoopSlot.kSlot1)
        //.velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        //.outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(4000)
        .maxAcceleration(4000)
        .allowedClosedLoopError(1)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot1);


    //motorConfig.inverted(true);

    motorConfig.softLimit
      .reverseSoftLimit(-180)
      .reverseSoftLimitEnabled(true) //have to enable soft limits for them to work ***
      .forwardSoftLimit(0)
      .forwardSoftLimitEnabled(true);
    
    motorConfig.limitSwitch
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchEnabled(true)
      .forwardLimitSwitchEnabled(false);

    
    climber2Motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(0); //RESETS THE ENCODER ON STARTUP TODO
  }

  public double climberGetCurrent(){
    return climber2Motor.getOutputCurrent();
  }

  public Command climberGoToPosition(double targetPos) {
    
    return runOnce(
        () -> {
          this.targetPos = targetPos;
          closedLoopController.setReference(targetPos, ControlType.kMAXMotionPositionControl);

        });
  } 

  public boolean climberAtPosition(){
    return Math.abs(getClimberPosition() - targetPos) < tolerance;
  }

  public Command climber2Spin(double speed){
    return Commands.runOnce(() -> {climber2Motor.set(speed);});
  }


  public double getClimberPosition(){
    return climber2Motor.getEncoder().getPosition();
  }

  public Command jiggle(double jiggleLength, double timesJiggled){
    return 
    runOnce(()-> {
      for(var i = 0; i < timesJiggled; i++){
        runOnce(() -> {
          targetPos = climber2Motor.getEncoder().getPosition() + jiggleLength;
          closedLoopController.setReference(targetPos, ControlType.kMAXMotionPositionControl);
        }).until(() -> 
          climberAtPosition()
        ).andThen(() -> {
          targetPos = climber2Motor.getEncoder().getPosition() - jiggleLength;
          closedLoopController.setReference(targetPos, ControlType.kMAXMotionPositionControl);  
        });
      }
    });
  }
  @Override
  public void periodic() {

    SmartDashboard.putNumber("climber current",climberGetCurrent());

    SmartDashboard.putBoolean("limit switch hit", limitSwitch.isPressed());
 
  }

  @Override
  public void simulationPeriodic() {
  }
}
