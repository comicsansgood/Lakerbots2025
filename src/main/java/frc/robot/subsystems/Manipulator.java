package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import au.grapplerobotics.LaserCan;

public class Manipulator extends SubsystemBase {

  //Subsystem objects
  public LaserCan lazer;
  public SparkMax manipulatorSpin;
  public SparkMax manipulatorWrist;
  public SparkClosedLoopController positionController;
  public SparkMaxConfig wristConfig;
  public SparkMaxConfig spinConfig;
  public SparkMaxConfig wristConfig2;
  private RelativeEncoder encoder;


  public boolean isCoralDetected;
  public boolean isAlgeaNotDetected;
  public double targetPos;
  public double setpoint;
  public double tolerance = 0.1;
  public double cutoffCurrent = 45;


  

  public Manipulator(){
    
    //objects
    lazer = new LaserCan(7);
    manipulatorSpin = new SparkMax(3, MotorType.kBrushless);
    manipulatorWrist = new SparkMax(4, MotorType.kBrushless);
    encoder = manipulatorWrist.getEncoder();
    positionController = manipulatorWrist.getClosedLoopController();
  
    //configs
    wristConfig = new SparkMaxConfig();
    spinConfig = new SparkMaxConfig();
    wristConfig2 = new SparkMaxConfig();

    wristConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    wristConfig2.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);


    wristConfig.idleMode(IdleMode.kBrake);
    wristConfig.smartCurrentLimit(40);

    wristConfig2.idleMode(IdleMode.kBrake);
    wristConfig2.smartCurrentLimit(40);

    spinConfig.smartCurrentLimit(40);
    

    wristConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.4)
        .i(.001) //added to make wrist hold position when at algea
        .d(0)
        .outputRange(-1, 1)
        .iZone(0.5); //untested might reduce overshoot

    //Bookmark-5
    wristConfig.closedLoop.maxMotion
        .maxVelocity(1500)
        .maxAcceleration(1500)
        .allowedClosedLoopError(.1);
  
        wristConfig.softLimit
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true) //*** have to enable soft limits for them to work ***
        .forwardSoftLimit(16)
        .forwardSoftLimitEnabled(true); //*** have to enable soft limits for them to work ***

        wristConfig2.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.4)
        .i(.001)
        .d(0)
        .outputRange(-1, 1)
        .iZone(0.5); 

    wristConfig2.closedLoop.maxMotion
        .maxVelocity(3000)
        .maxAcceleration(3000)
        .allowedClosedLoopError(.1);
  
        wristConfig2.softLimit
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true) //***have to enable soft limits for them to work ***
        .forwardSoftLimit(16)
        .forwardSoftLimitEnabled(true);//*** have to enable soft limits for them to work ***

    //comment swap this to flip between the two wrist config
    //manipulatorWrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    manipulatorWrist.configure(wristConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  
    manipulatorSpin.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(0);
  }



  public Command manipulatorSpinForTime(double speed, double time){
    return Commands.sequence(
      manipulatorSpin(speed),
      Commands.waitSeconds(time),
      manipulatorSpin(0)
    );
  }

  public Command manipulatorWristReset(){
    return runOnce(()->{
      encoder.setPosition(0);
    });
  }

  public Command spinUntilDetected(double speed) {
  return runEnd(
    () -> {
      manipulatorSpin.set(speed);
    },
    () -> {
      manipulatorSpin.set(0);
    }).until(() -> isCoralDetected == true);
  }

  public Command spinUntilNotDetected(double speed) {
    return runEnd(
      () -> {
        manipulatorSpin.set(speed);
      },
      () -> {
        manipulatorSpin.set(0);
      }).until(() -> isCoralDetected == false);
    }

  public Command manipulatorSpin(double speed){
    return Commands.runOnce(() -> {
      manipulatorSpin.set(speed);
    });
  }

public void manipulatorSpinMethod(double speed){
manipulatorSpin.set(speed);
}

  public Command manipulatorWristSpin(double speed){
    return Commands.runOnce(() -> {
      manipulatorWrist.set(speed);
    });
  }

  public Command manipulatorGoToPosition(double targetPos) {
    return runOnce(
        () -> {
          this.targetPos = targetPos;
          positionController.setReference(targetPos, ControlType.kMAXMotionPositionControl);
        });
  }

  public void setmanipulator(double targetPos){
    this.targetPos = targetPos;
    positionController.setReference(targetPos, ControlType.kMAXMotionPositionControl);
  }

  public Command manipulatorGoToPositionUntilThere(double targetPos){
    return runEnd(
      () -> {this.targetPos = targetPos;positionController.setReference(targetPos, ControlType.kMAXMotionPositionControl);},
      () -> {this.targetPos = targetPos;positionController.setReference(targetPos, ControlType.kMAXMotionPositionControl);}
      ).until(() -> manipulatorAtPosition());
  }

  public Command manipulatorSpinUntilCurrentReached(double speed, double holdSpeed){
    return runEnd(
      () -> {manipulatorSpin.set(speed);},
      () -> {manipulatorSpin.set(holdSpeed);}
      ).until(() -> manipulatorAtCurrent());
  }

  public Command manipulatorSpinUntilCurrentReachedWithWait(double speed, double holdspeed){
    return Commands.sequence(
      manipulatorSpin(speed),
      Commands.waitSeconds(0.5),
      manipulatorSpin(speed)
      );
  }

  public boolean manipulatorAtCurrent(){
    return getManipulatorCurrent() >= cutoffCurrent;
  }

  public boolean ismanipulatorClearToFlip(){
    return getManipulatorPosition() <= Constants.ManipulatorConstants.manipulatorClearToFlip;
  }

  public boolean manipulatorAtPosition(){
    return Math.abs(getManipulatorPosition() - targetPos) < tolerance;
  }
  public boolean manipulatorAtSpecificPosition(double setpoint){
    return Math.abs(getManipulatorPosition() - setpoint) < tolerance;
  }

  public double getManipulatorPosition(){
    return manipulatorWrist.getEncoder().getPosition();
  }

  public double getManipulatorCurrent(){
    return manipulatorSpin.getOutputCurrent();
  }
      
  public boolean exampleCondition() {
    return false;
  }

  @SuppressWarnings("static-access")
  @Override
  public void periodic() {

    //Smart Dashboard data output
    SmartDashboard.putBoolean("isLazerConnected", Constants.isLazerConnected);
    SmartDashboard.putBoolean("manipulatorAtPosition", manipulatorAtPosition());
    SmartDashboard.putNumber("manipulatorSpincurrent", getManipulatorCurrent());

    //try catch to not crash robot code when the lazer doesnt output
    try{
    isCoralDetected = lazer.getMeasurement().distance_mm <= 15 && lazer.getMeasurement().status == lazer.LASERCAN_STATUS_VALID_MEASUREMENT;/*  && lazer.getMeasurement().distance_mm != 0;*///changed from 30 to 15 b4 first comp
    SmartDashboard.putBoolean("isCoral", isCoralDetected);
    SmartDashboard.putNumber("lazer distance", lazer.getMeasurement().distance_mm);
    Constants.isLazerConnected = true;
    }catch(Exception e){
      Constants.isLazerConnected = false;
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}