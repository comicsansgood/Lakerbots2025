// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.PropertyNamingStrategy.PascalCaseStrategy;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.ComplexCommands;
import frc.robot.Commands.DriveTest;
import frc.robot.Commands.DriveToTarget;
import frc.robot.Commands.ElevatorMoveDownDynamic;
import frc.robot.Commands.ElevatorMoveDynamic;
import frc.robot.Commands.ElevatorMoveUpDynamic;
import frc.robot.Commands.CustomAutos.AutoCommand;
import frc.robot.Commands.CustomAutos.DriveToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Climber2;
import frc.robot.subsystems.FlapHook;
import frc.robot.subsystems.Leds;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.8).in(RadiansPerSecond);//0.75 ----> 0.8 3/15/25 mid western newengland // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    public static Manipulator manipulator = new Manipulator();
    public static Climber2 climber = new Climber2();
    public static FlapHook flapHook = new FlapHook();
    public static Elevator elevator = new Elevator();
    public static LimelightSubsystem limelight = new LimelightSubsystem();
    public static Leds leds = new Leds();
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        
        NamedCommands.registerCommand("L4Pose", ComplexCommands.scoreDynamic(4));
        NamedCommands.registerCommand("home", ComplexCommands.goToHomePose());
        NamedCommands.registerCommand("scoreCoral", manipulator.manipulatorSpinForTime(0.8, 0.75));// Time from 0.25 - 0.5 to then 0.75 --- SPEED increased from .35 to .5
        NamedCommands.registerCommand("collect", ComplexCommands.indexCoral());
        NamedCommands.registerCommand("algeaCollect", ComplexCommands.auto_algeaCollect());
        NamedCommands.registerCommand("algeaL2Pose", ComplexCommands.auto_goToAlgeaL2Pose());
        NamedCommands.registerCommand("algeaProcess", ComplexCommands.goToProcessorPoseDynamic());

        NamedCommands.registerCommand("elevatorGoDown", elevator.elevatorDownUntilThereDynamic(Constants.ElevatorConstants.elevatorHome));

        NamedCommands.registerCommand("manipulatorHome", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome));


        NamedCommands.registerCommand("scoreL4 Dynamic", ComplexCommands.scoreDynamic(4));

        NamedCommands.registerCommand("scoreL2", ComplexCommands.scoreDynamic(2));
        


        //new EventTrigger("triggerScoreL4").onTrue(ComplexCommands.score(4));
        new EventTrigger("triggerScoreL2").onTrue(ComplexCommands.scoreDynamic(2));
        //new EventTrigger("triggerHome").onTrue(ComplexCommands.goToHomePose());


        autoChooser = AutoBuilder.buildAutoChooser("Tests");




        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putData("tagtestyo", Commands.runOnce(()->AutoCommand.autoTest(drivetrain, limelight, robotSpeeds)));

        SmartDashboard.putData("drive to 0,0", new DriveToTarget(drivetrain, robotSpeeds,drivetrain.getState().Pose,
               new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
                MaxSpeed
                ));
        SmartDashboard.putData("drive to 1,1", new DriveToTarget(drivetrain, robotSpeeds,drivetrain.getState().Pose,
            new Pose2d(new Translation2d(1,1), new Rotation2d(0)),
            MaxSpeed
            ));

        SmartDashboard.putData("testing drive in command", new DriveTest(drivetrain, drive, robotSpeeds, MaxSpeed));

        SmartDashboard.putData("spin until detect", manipulator.spinUntilDetected(0.2));


        SmartDashboard.putData("driveToTag", new DriveToTag(drivetrain, limelight, robotSpeeds, Constants.TagConstants.tagTranslation));



        //so broken lol
        ///SmartDashboard.putData("climber down", climber.climber2Spin(0.2));
        ///SmartDashboard.putData("climber 0", climber.climber2Spin(0));
        //SmartDashboard.putData("climber up", climber.climber2Spin(-0.2));


        SmartDashboard.putData("manipulator wrist up", manipulator.manipulatorWristSpin(-0.05));
        SmartDashboard.putData("manipulator wrist 0", manipulator.manipulatorWristSpin(0.0));
        SmartDashboard.putData("manipulator wrist down", manipulator.manipulatorWristSpin(0.05));

        SmartDashboard.putData("manipulator forward", manipulator.manipulatorSpin(0.1));
        SmartDashboard.putData("manipulator 0", manipulator.manipulatorSpin(0));
        SmartDashboard.putData("manipulator backwards", manipulator.manipulatorSpin(-0.1));
      
        SmartDashboard.putData("flaphook Voltage Close", flapHook.flapHookSpin(-0.1));
        SmartDashboard.putData("flaphook 0", flapHook.flapHookSpin(0));
        SmartDashboard.putData("flaphook Voltage Open", flapHook.flapHookSpin(0.1));

        SmartDashboard.putData("Climber open loop go down", climber.climber2Spin(-0.15));
        SmartDashboard.putData("climbe open loop STOP", climber.climber2Spin(0));
        SmartDashboard.putData("climber open loop go up", climber.climber2Spin(0.15));

        SmartDashboard.putData("flaphook open", flapHook.hookGoToPosition(Constants.FlapHookConstants.hookflapOpen));
        SmartDashboard.putData("flaphook prepare", flapHook.hookGoToPosition(Constants.FlapHookConstants.hookPrepare));
        SmartDashboard.putData("flaphook closed", flapHook.hookGoToPosition(Constants.FlapHookConstants.hookLatch));
        SmartDashboard.putData("Reset Flap Hook Encoder",flapHook.flapHookEncoderReset());


        SmartDashboard.putData("climber position HOME", climber.climberGoToPosition(Constants.ClimberConstants.climberHome));
        SmartDashboard.putData("climber position down", climber.climberGoToPosition(Constants.ClimberConstants.climberDown));
        SmartDashboard.putData("climber position mid", climber.climberGoToPosition(Constants.ClimberConstants.climberMid));


        SmartDashboard.putData("manipulator algea", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorAlgeaCollect));
        SmartDashboard.putData("manipulator home", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorHome));
        SmartDashboard.putData("manipulator travel", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorTravel));
        SmartDashboard.putData("manipulator L1", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorCoralL1));
        SmartDashboard.putData("manipulator L4", manipulator.manipulatorGoToPosition(Constants.ManipulatorConstants.manipulatorCoralL4));

       
        SmartDashboard.putData("elevator home", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorHome));
        SmartDashboard.putData("elevator process", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorProcess));
        SmartDashboard.putData("elevator coral 1", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCoralL1));
        SmartDashboard.putData("elevator coral 2", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCoralL2));
        SmartDashboard.putData("elevator coral 3", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCoralL3));
        SmartDashboard.putData("elevator coral 4", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorCoralL4));
        SmartDashboard.putData("elevator alegae2", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL2));
        SmartDashboard.putData("elevator alegae3", elevator.elevatorGoToPosition(Constants.ElevatorConstants.elevatorAlgaeL3));


        //SmartDashboard.putData("center coral", ComplexCommands.centerCoral());
        //SmartDashboard.putData("travel sequence test", ComplexCommands.travelSequence());
        SmartDashboard.putData("elevator go home sequence", ComplexCommands.goToHomePose());
        SmartDashboard.putData("indexCoral", ComplexCommands.indexCoral());
        SmartDashboard.putData("manipulatorSpinfortime", manipulator.manipulatorSpinForTime(0.2,3));
        SmartDashboard.putData("scoretest (1)", ComplexCommands.score(1));
        SmartDashboard.putData("scoretest (2)", ComplexCommands.score(2));
        SmartDashboard.putData("scoretest (3)", ComplexCommands.score(3));
        SmartDashboard.putData("scoretest (4)", ComplexCommands.score(4));
        SmartDashboard.putData("manipualtorspinuntilcurrent", manipulator.manipulatorSpinUntilCurrentReached(-0.3,-0.1));
        SmartDashboard.putData("goToProccesserPose", ComplexCommands.goToProcessorPose());
        SmartDashboard.putData("collectAlg2", ComplexCommands.collectAlgeaL2());
        SmartDashboard.putData("collectAlg3", ComplexCommands.collectAlgeaL3());

        SmartDashboard.putData("homeDynamic", ComplexCommands.goToHomePoseDynamic());
        SmartDashboard.putData("L1 Dynamic", ComplexCommands.scoreDynamic(1));
        SmartDashboard.putData("L2 Dynamic", ComplexCommands.scoreDynamic(2));
        SmartDashboard.putData("L3 Dynamic", ComplexCommands.scoreDynamic(3));
        SmartDashboard.putData("L4 Dynamic", ComplexCommands.scoreDynamic(4));

        SmartDashboard.putData("dynamic logic test L1", new ElevatorMoveDynamic(elevator, Constants.ElevatorConstants.elevatorCoralL1));
        SmartDashboard.putData("dynamic logic test L2", new ElevatorMoveDynamic(elevator, Constants.ElevatorConstants.elevatorCoralL2));
        SmartDashboard.putData("dynamic logic test L3", new ElevatorMoveDynamic(elevator, Constants.ElevatorConstants.elevatorCoralL3));
        SmartDashboard.putData("dynamic logic test L4", new ElevatorMoveDynamic(elevator, Constants.ElevatorConstants.elevatorCoralL4));





        /*SmartDashboard.putData("Elevator up dynamic", ComplexCommands.elevatorGoUp(Constants.ElevatorConstants.elevatorCoralL3));
        SmartDashboard.putData("Elevator down dynamic", ComplexCommands.elevatorGoDown(Constants.ElevatorConstants.elevatorHome));
        SmartDashboard.putData("Elevator up dynamicBetter", ComplexCommands.elevatorGoUp(Constants.ElevatorConstants.elevatorCoralL3));*/

        SmartDashboard.putData("elevator up dynamic best", new ElevatorMoveUpDynamic(elevator, Constants.ElevatorConstants.elevatorCoralL4));
        SmartDashboard.putData("elevator down dynamic best", new ElevatorMoveDownDynamic(elevator, Constants.ElevatorConstants.elevatorHome));




        //test


        SmartDashboard.putData("manipulatorReset", manipulator.manipulatorWristReset());
        SmartDashboard.putNumber("Elevator Current", elevator.elevatorGetCurrent());


        SmartDashboard.putData("led spirit", Commands.runOnce(()->{leds.spiritColors();}));
        SmartDashboard.putData("black", Commands.runOnce(()->{leds.black();}));
        SmartDashboard.putData("red", Commands.runOnce(()->{leds.red();}));
        SmartDashboard.putData("rainfall", Commands.runOnce(()->{leds.rainfallSeqence();}));




        //SmartDashboard.putNumber("climber current", climber.climberGetCurrent());

        /*SmartDashboard.putData("testingDrivecmd", drivetrain.applyRequest(() ->
        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));*/

        configureBindings();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        

    
        driverJoystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(1).withVelocityY(0))
        );
        driverJoystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-1).withVelocityY(0))
        );
        driverJoystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-1))
        );
        driverJoystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(1))
        );
    
        
  

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/


        // reset the field-centric heading on left bumper press

        driverJoystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


        driverJoystick.leftBumper().onTrue(ComplexCommands.indexCoral());
        driverJoystick.rightBumper().onTrue(manipulator.manipulatorSpinForTime(0.2, 1));
        driverJoystick.x().onTrue(manipulator.manipulatorSpinForTime(-0.1, 0.5));
        driverJoystick.y().onTrue(flapHook.hookGoToPosition(Constants.FlapHookConstants.hookLatch));
        driverJoystick.a().onTrue(climber.climberGoToPosition(Constants.ClimberConstants.climberDown));
        //driverJoystick.b().onTrue(climber.climberGoToPosition(Constants.ClimberConstants.climberHome));
        driverJoystick.b().onTrue(manipulator.manipulatorSpinForTime(0.07, 0.375));



        operatorJoystick.a().onTrue(ComplexCommands.goToProcessorPoseDynamic());
        operatorJoystick.x().onTrue(ComplexCommands.scoreDynamic(2));
        operatorJoystick.y().onTrue(ComplexCommands.scoreDynamic(3));
        operatorJoystick.b().onTrue(ComplexCommands.scoreDynamic(4));
        operatorJoystick.leftBumper().onTrue(ComplexCommands.goToHomePoseDynamic().andThen(ComplexCommands.indexCoral()));
        operatorJoystick.rightBumper().onTrue(flapHook.hookGoToPosition(Constants.FlapHookConstants.hookPrepare));
        //operatorJoystick.rightBumper().onTrue(ComplexCommands.goToProcessorPose());
        operatorJoystick.back().onTrue(ComplexCommands.collectAlgeaL2Dynamic());
        operatorJoystick.start().onTrue(ComplexCommands.collectAlgeaL3Dynamic());

        /*operatorJoystick.pov(0).whileTrue(flapHook.flapHookSpin(0.2));
        operatorJoystick.pov(180).whileTrue(flapHook.flapHookSpin(-0.2));
        operatorJoystick.povCenter().onTrue(flapHook.flapHookSpin(0.0));*/

        


        
    }
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
