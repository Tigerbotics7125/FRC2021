package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

class MotorPosition {
    private WPI_TalonSRX motor;
    private int loops;
    private boolean startSW;
    private boolean endSW;
    private int startPos;
    private int endPos;
    private boolean isSetUp;

    public MotorPosition(int port) {
        isSetUp=false;
        motor = new WPI_TalonSRX(port);
 
        startSW = motor.getSensorCollection().isRevLimitSwitchClosed();
        endSW = motor.getSensorCollection().isFwdLimitSwitchClosed();
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        /* Ensure sensor is positive when output is positive */
        motor.setSensorPhase(Constants.kSensorPhase);
        loops = 0;
        /**
         * Set based on what direction you want forward/positive to be. This does not
         * affect sensor phase.
         */
        motor.setInverted(Constants.kMotorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        motor.configNominalOutputForward(0, Constants.kTimeoutMs);
        motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
        motor.configPeakOutputForward(1, Constants.kTimeoutMs);
        motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /**
         * Config the allowable closed-loop error, Closed-Loop output will be neutral
         * within this range. See Table in Section 17.2.1 for native units per rotation.
         */
        motor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        /* Config Position Closed Loop gains in slot0, typically kF stays zero. */
        motor.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        motor.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        motor.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        motor.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

        /**
         * Grab the 360 degree position of the MagEncoder's absolute position, and
         * intitally set the relative sensor to match.
         */
        int absolutePosition = motor.getSensorCollection().getPulseWidthPosition();

        /* Mask out overflows, keep bottom 12 bits */
        absolutePosition &= 0xFFF;
        if (Constants.kSensorPhase) {
            absolutePosition *= -1;
        }
        if (Constants.kMotorInvert) {
            absolutePosition *= -1;
        }

        /* Set the quadrature (relative) sensor to match absolute */
        motor.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
       
    }

    public void setStart(){
         //Find the position relating to the start
        while (!startSW) {
            motor.set(ControlMode.Position, motor.getSelectedSensorPosition(0) - .1);
            startSW = motor.getSensorCollection().isRevLimitSwitchClosed();
        }
        startPos = motor.getSelectedSensorPosition(0);
        //Find the position relating to the end
        while (!endSW) {
            motor.set(ControlMode.Position, motor.getSelectedSensorPosition(0) + .1);
            endSW = motor.getSensorCollection().isFwdLimitSwitchClosed();
        }
        endPos = motor.getSelectedSensorPosition(0);
        isSetUp=true;
    }
    public boolean getSetUp(){
        return isSetUp;
    }
    public void move(double analogControl, double divider) {
        /* Get Talon/Victor's current output percentage */
        double motorOutput = motor.getMotorOutputPercent();
        StringBuilder _sb = new StringBuilder();
        /* Deadband gamepad */
        if (Math.abs(analogControl) < 0.10) {
            /* Within 10% of zero */
            analogControl = 0;
        }

        /* Prepare line to print */
        _sb.append("\tout:");
        /* Cast to int to remove decimal places */
        _sb.append((int) (motorOutput * 100));
        _sb.append("%"); // Percent

        _sb.append("\tpos:");
        _sb.append(motor.getSelectedSensorPosition(0));
        _sb.append("u"); // Native units

        /**
         * When button 1 is pressed, perform Position Closed Loop to selected position,
         * indicated by Joystick position x10, [-10, 10] rotations
         */
        // if (!_lastButton1 && button1) {
        /* Position Closed Loop */

        /* Half Rotation * 4096 u/rev in either direction */
        double range=startPos-endPos;
        double targetPositionRotations = analogControl * range +startPos;
        motor.set(ControlMode.Position, targetPositionRotations);
        // }

        /* If Talon is in position closed-loop, print some more info */
        if (motor.getControlMode() == ControlMode.Position) {
            /* ppend more signals to print when in speed mode. */
            _sb.append("\terr:");
            _sb.append(motor.getClosedLoopError(0));
            _sb.append("u"); // Native Units

            _sb.append("\ttrg:");
            _sb.append(targetPositionRotations);
            _sb.append("u"); /// Native Units
        }

        /**
         * Print every ten loops, printing too much too fast is generally bad for
         * performance.
         */
        if (++loops >= 10) {
            loops = 0;
            System.out.println(_sb.toString());
        }

        /* Reset built string for next loop */
        _sb.setLength(0);
        // hood.set(ControlMode.PercentOutput, analogControl * divider);
    }

}