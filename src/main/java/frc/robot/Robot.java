/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Joystick gamepad;

  private WPI_VictorSPX leftMotor;
  private WPI_VictorSPX rightMotor;
  private DifferentialDrive chassis;

  private WPI_TalonSRX uptakeOne;
  private WPI_TalonSRX hood;
  private WPI_TalonSRX shooter;

  private Spark intake;
  private Spark uptakeTwo;
  private Spark uptakeThree;
  private Spark drawBridge;

  private DigitalInput drawBridgeDown;

  private boolean index = false;
  private double indexStart;



  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    gamepad = new Joystick(0);

    leftMotor = new WPI_VictorSPX(0);
    rightMotor = new WPI_VictorSPX(1);
    chassis = new DifferentialDrive(leftMotor, rightMotor);

    uptakeOne = new WPI_TalonSRX(2);
    hood = new WPI_TalonSRX(3);
    shooter = new WPI_TalonSRX(4);

    intake = new Spark(0);
    uptakeTwo = new Spark(1);
    uptakeThree = new Spark(2);
    drawBridge = new Spark(3);

    drawBridgeDown = new DigitalInput(0);

   
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }

    // lower drawbridge until its down
    if (!drawBridgeDown.get()) {
      drawBridge.set(-1);
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // get current gamepad state
    boolean aButton = gamepad.getRawButton(1);
    boolean bButton = gamepad.getRawButton(2);
    boolean xButton = gamepad.getRawButton(3);
    boolean yButton = gamepad.getRawButton(4);
    boolean leftBumper = gamepad.getRawButton(5);
    boolean rightBumper = gamepad.getRawButton(6);
    double leftXAxis = gamepad.getRawAxis(0); // left negitive, right positive
    double leftYAxis = gamepad.getRawAxis(1); // up negitive, down positive
    double leftTrigger = gamepad.getRawAxis(2); // no negitive
    double rightTrigger = gamepad.getRawAxis(3); // no negitive
    double rightXAxis = gamepad.getRawAxis(4); // left negitive, right positive
    double rightYAxis = gamepad.getRawAxis(5); // up negitive, down positive

    double leftXAxisWDeadzone = 0;
    double leftYAxisWDeadzone = 0;
    double rightXAxisWDeadzone = 0;
    double rightYAxisWDeadzone = 0;

    // deadzone
    if (leftXAxis < .1 && leftXAxis > -.1) {
      leftXAxisWDeadzone = 0;
    } else {
      leftXAxisWDeadzone = leftXAxis;
    }
    if (leftYAxis < .1 && leftYAxis > -.1) {
      leftYAxisWDeadzone = 0;
    } else {
      leftYAxisWDeadzone = leftYAxis;
    }
    if (rightXAxis < .1 && rightXAxis > -.1) {
      rightXAxisWDeadzone = 0;
    } else {
      rightXAxisWDeadzone = rightXAxis;
    }
    if (rightYAxis < .1 && rightYAxis > -.1) {
      rightYAxisWDeadzone = 0;
    } else {
      rightYAxisWDeadzone = rightYAxis;
    }


    // drawbridge
/*
    if (yButton) {
      drawBridge.set(1);
    }

    if (xButton && drawBridgeDown.get()) {
      drawBridge.set(-1);
    }
*/
    // intake
/*  if (aButton) {
      intake.set(.25);
    }
*/
    // index
    if (!index && bButton) {
      index = true;
      indexStart = Timer.getFPGATimestamp();
    }
    if (aButton) {
      uptakeOne.set(ControlMode.PercentOutput, 1);
      intake.set(-.25);
    } else {
      uptakeOne.set(ControlMode.PercentOutput, 0);
      intake.set(0);
    }
    if (rightBumper || index) {
      uptakeTwo.set(-1);
    } else {
      uptakeTwo.set(0);
    }
    if (leftBumper || index) {
      uptakeThree.set(-1);
    } else {
      uptakeThree.set(0);
    }

    double curTime = Timer.getFPGATimestamp();
    if (curTime > indexStart + 4 && index) {
      index = false;
    }

    // shooter

    shooter.set(ControlMode.PercentOutput, rightTrigger*.5);

    // hood

    hood.set(ControlMode.PercentOutput, rightXAxisWDeadzone*.25);
    
    // chassis / drivechain

    //chassis.arcadeDrive(leftYAxisWDeadzone, leftXAxisWDeadzone);
    chassis.arcadeDrive(0, 0);

  }
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
