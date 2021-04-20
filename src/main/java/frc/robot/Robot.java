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
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Autonomous
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Cameras
  private UsbCamera camera1;
  private UsbCamera camera2;

  // Victor
  private WPI_VictorSPX leftMotor;
  private WPI_VictorSPX rightMotor;

  // Talon
  private WPI_TalonSRX uptakeOne;
  private WPI_TalonSRX hood;
  private WPI_TalonSRX shooter;
  private WPI_TalonSRX pigeonTalon;

  // Spark
  private Spark intake;
  private Spark uptakeTwo;
  private Spark uptakeThree;
  private Spark drawBridge;

  // WPI
  private DifferentialDrive chassis;
  private DigitalInput drawBridgeDown;

  // primitives
  private boolean index;
  private double indexStart;

  // gamepad
  private Joystick gamepad;
  private boolean aButton;
  private boolean bButton;
  private boolean xButton;
  private boolean yButton;
  private boolean leftBumper;
  private boolean rightBumper;
  private double leftXAxis; // left negative, right positive
  private double leftYAxis; // up negative, down positive
  private double leftTrigger; // no negative
  private double rightTrigger; // no negative
  private double rightXAxis; // left negative, right positive
  private double rightYAxis; // up negative, down positive
  private double leftXAxisWDeadzone;
  private double leftYAxisWDeadzone;
  private double rightXAxisWDeadzone;
  private double rightYAxisWDeadzone;

  // other
  private PigeonIMU pigeon;

  private int _loops;
  @Override
  public void robotInit() {
    // Autonomous
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Cameras
    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera2 = CameraServer.getInstance().startAutomaticCapture(1);

    gamepad = new Joystick(0);

    // Victors
    leftMotor = new WPI_VictorSPX(0);
    rightMotor = new WPI_VictorSPX(1);

    // Talon
    uptakeOne = new WPI_TalonSRX(2);

    hood = new WPI_TalonSRX(3);
    setUpHood();

    shooter = new WPI_TalonSRX(4);
    pigeonTalon = new WPI_TalonSRX(10);

    // Spark
    intake = new Spark(0);
    uptakeTwo = new Spark(1);
    uptakeThree = new Spark(2);
    drawBridge = new Spark(3);

    // WPI
    chassis = new DifferentialDrive(leftMotor, rightMotor);
    drawBridgeDown = new DigitalInput(0);

    // primitives

    // gamepad

    // other
    pigeon = new PigeonIMU(pigeonTalon);
    pigeon.setYaw(0.0);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // lower drawbridge until down
      do {
        drawBridge.set(-1);
      } while (!drawBridgeDown.get());
      // STOP WHEN DOWN
      drawBridge.set(0);

      break;
    case kDefaultAuto:
    default:
      // lower drawbridge until down
      do {
        drawBridge.set(-1);
      } while (!drawBridgeDown.get());
      // STOP WHEN DOWN
      drawBridge.set(0);

      break;
    }

  }

  @Override
  public void teleopPeriodic() {

    // Pigeon
    getAndSendPigeonStatus();

    // gamepad & motors
    updateGamepadStatus();
    drawbridge(yButton, xButton); // drawbridge up, drawbridge down
    indexBall(bButton, aButton, rightBumper, leftBumper, leftTrigger); // startIndex, intake, uptakeTwo, uptakeThree,
                                                                       // backout
    shooter(rightTrigger, 1); // analogControl, divider
    hood(rightXAxisWDeadzone, .25); // analogControl, divider
    arcadeDrive(leftYAxisWDeadzone, leftXAxisWDeadzone, .5); // drive, rotate, divider
  }

  private void arcadeDrive(double drive, double rotate, double divider) {
    chassis.arcadeDrive(drive * divider, rotate * divider);
  }

  private void setUpHood(){
    hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
    /* Ensure sensor is positive when output is positive */
    hood.setSensorPhase(Constants.kSensorPhase);
    _loops=0;
    /**
     * Set based on what direction you want forward/positive to be. This does not
     * affect sensor phase.
     */
    hood.setInverted(Constants.kMotorInvert);

    /* Config the peak and nominal outputs, 12V means full */
    hood.configNominalOutputForward(0, Constants.kTimeoutMs);
    hood.configNominalOutputReverse(0, Constants.kTimeoutMs);
    hood.configPeakOutputForward(1, Constants.kTimeoutMs);
    hood.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    hood.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    hood.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    hood.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    hood.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    hood.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /**
     * Grab the 360 degree position of the MagEncoder's absolute position, and
     * intitally set the relative sensor to match.
     */
    int absolutePosition = hood.getSensorCollection().getPulseWidthPosition();

    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    if (Constants.kSensorPhase) {
      absolutePosition *= -1;
    }
    if (Constants.kMotorInvert) {
      absolutePosition *= -1;
    }

    /* Set the quadrature (relative) sensor to match absolute */
    hood.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

  }
  private void hood(double analogControl, double divider) {
    /* Get Talon/Victor's current output percentage */
		double motorOutput = hood.getMotorOutputPercent();
    StringBuilder _sb=new StringBuilder();
		/* Deadband gamepad */
		if (Math.abs(analogControl) < 0.10) {
			/* Within 10% of zero */
			analogControl = 0;
		}

		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent

		_sb.append("\tpos:");
		_sb.append(hood.getSelectedSensorPosition(0));
		_sb.append("u"); 	// Native units

		/**
		 * When button 1 is pressed, perform Position Closed Loop to selected position,
		 * indicated by Joystick position x10, [-10, 10] rotations
		 */
	//	if (!_lastButton1 && button1) {
			/* Position Closed Loop */

			/* 10 Rotations * 4096 u/rev in either direction */
			double targetPositionRotations = analogControl * 10.0 * 4096;
			hood.set(ControlMode.Position, targetPositionRotations);
		//}


		/* If Talon is in position closed-loop, print some more info */
		if (hood.getControlMode() == ControlMode.Position) {
			/* ppend more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(hood.getClosedLoopError(0));
			_sb.append("u");	// Native Units

			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations);
			_sb.append("u");	/// Native Units
		}

		/**
		 * Print every ten loops, printing too much too fast is generally bad
		 * for performance.
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Reset built string for next loop */
		_sb.setLength(0);
//    hood.set(ControlMode.PercentOutput, analogControl * divider);
  }

  private void shooter(double analogControl, double divider) {
    shooter.set(ControlMode.PercentOutput, analogControl * divider);
  }

  public void indexBall(boolean startIndexButton, boolean intakeButton, boolean manualUptakeTwoButton,
      boolean manualUptakeThreeButton, double analogBackoutButton) {
    // intake, part of indexing, but not controlled by timer.
    if (intakeButton) {
      uptakeOne.set(ControlMode.PercentOutput, 1);
      intake.set(-.25);
    } else {
      uptakeOne.set(ControlMode.PercentOutput, 0);
      intake.set(0);
    }

    // if you press the index button, and not currently index, start indexing and
    // set the initial time to now.
    if (!index && startIndexButton) {
      index = true;
      indexStart = Timer.getFPGATimestamp();
    }

    // uptakeTwo
    if (index || manualUptakeTwoButton) {
      uptakeTwo.set(-1);
    } else {
      uptakeTwo.set(-1 * analogBackoutButton);
    }

    // UptakeThree
    if (index || manualUptakeThreeButton) {
      uptakeThree.set(-1);
    } else {
      uptakeThree.set(analogBackoutButton);
    }

    // stop index if time is up
    double curTime = Timer.getFPGATimestamp();
    if (curTime > indexStart + 4 && index) {
      index = false;
    }
  }

  private void drawbridge(boolean upButton, boolean downButton) {
    if (upButton) {
      drawBridge.set(1);
    } else if (downButton && drawBridgeDown.get()) {
      drawBridge.set(-1);
    } else {
      drawBridge.set(0);
    }
  }

  private void getAndSendPigeonStatus() {
    // setting up values
    double[] ypr = new double[3];
    double[] accelxyz = new double[3];
    double[] gyroxyz = new double[3];
    double compass;
    // setting values
    pigeon.getYawPitchRoll(ypr);
    pigeon.getAccelerometerAngles(accelxyz);
    pigeon.getRawGyro(gyroxyz);
    compass = pigeon.getCompassFieldStrength();

    // send values to SmartDashboard, both as an array and values
    // ypr
    SmartDashboard.putNumberArray("Yaw Pitch Roll", ypr);
    SmartDashboard.putNumber("Yaw", ypr[0]);
    SmartDashboard.putNumber("Pitch", ypr[1]);
    SmartDashboard.putNumber("Roll", ypr[2]);
    // accel
    SmartDashboard.putNumberArray("Accelerometer", accelxyz);
    SmartDashboard.putNumber("X Acceleration", accelxyz[0]);
    SmartDashboard.putNumber("Y Acceleration", accelxyz[1]);
    SmartDashboard.putNumber("Z Acceleration", accelxyz[2]);
    // gyro
    SmartDashboard.putNumberArray("Gyroscope", gyroxyz);
    SmartDashboard.putNumber("X Rotation", gyroxyz[0]);
    SmartDashboard.putNumber("Y Rotation", gyroxyz[1]);
    SmartDashboard.putNumber("Z Rotation", gyroxyz[2]);
    // compass
    SmartDashboard.putNumber("Compass", compass);
  }

  private void updateGamepadStatus() {
    aButton = gamepad.getRawButton(1);
    bButton = gamepad.getRawButton(2);
    xButton = gamepad.getRawButton(3);
    yButton = gamepad.getRawButton(4);
    leftBumper = gamepad.getRawButton(5);
    rightBumper = gamepad.getRawButton(6);
    leftXAxis = gamepad.getRawAxis(0); // left negitive, right positive
    leftYAxis = gamepad.getRawAxis(1); // up negitive, down positive
    leftTrigger = gamepad.getRawAxis(2); // no negitive
    rightTrigger = gamepad.getRawAxis(3); // no negitive
    rightXAxis = gamepad.getRawAxis(4); // left negitive, right positive
    rightYAxis = gamepad.getRawAxis(5); // up negitive, down positive

    setDeadzoneValues();
  }

  private void setDeadzoneValues() {
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
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
