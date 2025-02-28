// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Climber2;
import frc.robot.subsystems.FlapHook;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {


        NamedCommands.registerCommand("scoreL4", ComplexCommands.score(4));

        NamedCommands.registerCommand("scoreL2", ComplexCommands.score(2));
        NamedCommands.registerCommand("home", ComplexCommands.goToHomePose());
        NamedCommands.registerCommand("scoreCoral", manipulator.manipulatorSpinForTime(0.1, 1));

        //new EventTrigger("triggerScoreL4").onTrue(ComplexCommands.score(4));
        //new EventTrigger("triggerScoreL2").onTrue(ComplexCommands.score(2));
        //new EventTrigger("triggerHome").onTrue(ComplexCommands.goToHomePose());


        autoChooser = AutoBuilder.buildAutoChooser("Tests");




        SmartDashboard.putData("Auto Mode", autoChooser);

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
        SmartDashboard.putData("Elevator up dynamic", ComplexCommands.elevatorGoUp(Constants.ElevatorConstants.elevatorCoralL3));
        SmartDashboard.putData("Elevator down dynamic", ComplexCommands.elevatorGoDown(Constants.ElevatorConstants.elevatorHome));
        SmartDashboard.putData("Elevator up dynamicBetter", ComplexCommands.elevatorGoUp(Constants.ElevatorConstants.elevatorCoralL3));




        //test


        SmartDashboard.putData("manipulatorReset", manipulator.manipulatorWristReset());
        SmartDashboard.putNumber("Elevator Current", elevator.elevatorGetCurrent());

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
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverJoystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        driverJoystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


        driverJoystick.leftBumper().onTrue(ComplexCommands.indexCoral());
        driverJoystick.rightBumper().onTrue(manipulator.manipulatorSpinForTime(0.1, 1));

        operatorJoystick.a().onTrue(ComplexCommands.goToProcessorPose());
        operatorJoystick.x().onTrue(ComplexCommands.score(2));
        operatorJoystick.y().onTrue(ComplexCommands.score(3));
        operatorJoystick.b().onTrue(ComplexCommands.score(4));
        operatorJoystick.leftBumper().onTrue(ComplexCommands.goToHomePose());
        //operatorJoystick.rightBumper().onTrue(ComplexCommands.goToProcessorPose());
        operatorJoystick.back().onTrue(ComplexCommands.collectAlgeaL2());
        operatorJoystick.start().onTrue(ComplexCommands.collectAlgeaL3());
        


        
    }
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
