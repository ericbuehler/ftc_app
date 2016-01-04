package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by ericbuehler on 1/3/16.
 */
public class T6288ManOp extends OpMode {

    //Define some thresholds to limit how the robot moves
    final static double MAX_MOTOR_PWR = 1.0; //Never let the motor go beyond this
    final static double MIN_MOTOR_PWR = -1.0; //Never let the motor go beyond this
    final static double MAX_MOTOR_PWR_SOFT = 0.4; //This is our soft limit (without override)
    final static double MIN_MOTOR_PWR_SOFT = -0.4; //This is our soft limit (without override)
    final static double MAX_DRIVE_PWR_RATIO_SOFT = MAX_MOTOR_PWR_SOFT; //We shouldn't drive any faster than this
    final static double MIN_DRIVE_PWR_RATIO_SOFT = MIN_MOTOR_PWR_SOFT; //We shouldn't drive any faster than this

    //For joystick centering threshold
    final static double MAX_ABS_JOY_ZERO = 0.05; //This value is close enough to zero

    //For servo reset position
    final static double ROBOT_LIFT_SERVO_LEFT_START = 0.0; //TODO: Make sure this is correct
    final static double ROBOT_LIFT_SERVO_RIGHT_START = 0.0; //TODO: Make sure this is correct

    //We want to add a timer here so that we can see how much time we have left
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();

    //Need to have some variables to control the robot
    DcMotor driverMotor;
    DcMotor passengerMotor;
    DcMotor armLiftMotor;
    DcMotor armExtendMotor1;
    DcMotor armExtendMotor2;
    Servo robotLiftServoLeft;
    Servo robotLiftServoRight;
    DcMotorController driveController;

    final static String driverMotorName = "driveleft";
    final static String passengerMotorName = "driveright";
    final static String driveControllerName = "Motor Controller 1";
    final static String armLiftMotorName = "armup";
    final static String armLiftControllerName = "mc0";
    final static String armExtendMotor1Name = "armout1";
    final static String armExtendMotor2Name = "armout2";
    final static String armExtendControllerName = "mc5";
    final static String robotLiftServoLeftName = "liftleft";
    final static String robotLiftServoRightName = "liftright";
    final static String robotLiftControllerName = "sc1";


    //For driving motors
    double drivePower = 0.0;
    double driveBalance = 0.0;

    //For driving the arms
    boolean selectArm = false;
    double driveArmExtend = 0;
    double driveArmLift = 0;
    float armRaise = 0;
    float armLower = 0;
    boolean armExtend = false;
    boolean armRetract = false;

    //For driving the lift
    boolean servoReset = false;
    boolean servoUp = false;
    boolean servoDown = false;

//TODO: Need to add telemetry options

    /*
     * This function will read the whole state of the robot layout
     */
    public void robot_read_layout() {
        //Initialize all of objects for robot devices
        //Driving the robot
        //Motors
        driverMotor = hardwareMap.dcMotor.get(driverMotorName);
        driverMotor.setDirection(DcMotor.Direction.FORWARD); //TODO: Make sure it is correct
        passengerMotor = hardwareMap.dcMotor.get(passengerMotorName);
        passengerMotor.setDirection(DcMotor.Direction.FORWARD); //TODO: Make sure it is correct
//        //Motor Controllers
//        driveController = hardwareMap.dcMotorController.get(driveControllerName);
        //Moving the arm
        armLiftMotor = hardwareMap.dcMotor.get(armLiftMotorName);
        armLiftMotor.setDirection(DcMotor.Direction.FORWARD); //TODO: Make sure it is correct
        armExtendMotor1 = hardwareMap.dcMotor.get(armExtendMotor1Name);
        armExtendMotor1.setDirection(DcMotor.Direction.FORWARD); //TODO: Make sure it is correct
        armExtendMotor2 = hardwareMap.dcMotor.get(armExtendMotor2Name);
        armExtendMotor2.setDirection(DcMotor.Direction.FORWARD); //TODO: Make sure it is correct
        //Lifting the robot
        robotLiftServoLeft = hardwareMap.servo.get(robotLiftServoLeftName);
        robotLiftServoLeft.setDirection(Servo.Direction.FORWARD); //TODO: Make sure it is correct
        robotLiftServoRight = hardwareMap.servo.get(robotLiftServoRightName);
        robotLiftServoRight.setDirection(Servo.Direction.FORWARD); //TODO: Make sure it is correct
    }

    /*
     * This function will initialize the robot (this is mostly used for arms)
     */
    public void servo_state_reset() {
        robotLiftServoLeft.setPosition(ROBOT_LIFT_SERVO_LEFT_START);
        robotLiftServoRight.setPosition(ROBOT_LIFT_SERVO_RIGHT_START);
    }

    /*
     * This function will initialize the robot (this is mostly used for arms)
     */
    public void robot_state_reset() {
        servo_state_reset();
    }

    /*
     * This function is used for driving the robot
     */
    public void set_robot_drive(double joyPower, double joyBalance) {
        //First set the power level according to the zero threshold
        if (Math.abs(joyPower) > MAX_ABS_JOY_ZERO) {
            //Set the value to the max allowed
            joyPower *= MAX_DRIVE_PWR_RATIO_SOFT;
        } else {
            joyPower = 0;
        }

        //Set power value for each motor
        double driverMotorPwr = joyPower;
        double passengerMotorPwr = joyPower;

        //Now balance out the power for turning
        //If the Balance is 1 then all driver
        //If the Balance is -1 then all passenger
        if (Math.abs(joyBalance) > MAX_ABS_JOY_ZERO) { //The robot is within the zero threshold
            if (joyBalance > 0) { //focus on driver skew
                passengerMotorPwr = (1 - joyBalance) * passengerMotorPwr;
            } else { //All passenger
                driverMotorPwr = (1 - Math.abs(joyBalance)) * driverMotorPwr;
            }
        }

        //Do some driving
        driverMotor.setPower(driverMotorPwr);
        passengerMotor.setPower(passengerMotorPwr);
    }

    /*
     * This function will drive the arm
     */
    public void set_arm_drive(double armLift, double armExtend, boolean killArm = false) {
        //TODO: will need to figure out how to keep the arm at a single position

        //This shuts down the motors (saves battery)
        if (killArm) {
            //TODO: Need to reset the hold variable to nothing
            armLiftMotor.setPower(0.0);
        } else { //You really want to do some lifting
            armLiftMotor.
        }
    }

    /*
     * This function is pretty simple to move the servo in a direction to lift
     * the robot up
     */
    public void set_servo_drive(boolean servoUp = false) {
        if (servoUp) {

        } else {

        }
    }
    /*
     * This function will stop all robot activity call this in init and stop
     */
    public void robot_halt() {
        //Stop all driving motors
        set_robot_drive(0,0);
        //Stop all arm motors
        set_arm_drive(0,0,1);
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        telemetry.addData("Mode: ", "init");
        robot_read_layout(); //Set up the objects
        robot_halt(); //Turn off any running motors
        robot_state_reset(); //Reset the servos
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
        //Initialize the time
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        //Print out the start time
        telemetry.addData("Mode: ", "init_loop, " + "Rt:" + runtime.toString() + " " + startDate);
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        telemetry.addData("RunTime: ", runtime.toString());

        //Get the joystick values
        drivePower = -gamepad1.left_stick_y; //if y equal -1 then joy is pushed all the way forward
        driveBalance = gamepad1.right_stick_x; //0 is equal power distribution between motors
        armRaise = gamepad1.right_trigger; //Use this to lift the arm
        armLower = gamepad1.left_trigger; //Use this to lower the arm
        armExtend = gamepad1.right_bumper; //Use this to extend arm1 or arm2
        armRetract = gamepad1.left_bumper; //Use this to retract arm1 or arm2
        selectArm = gamepad1.a; //Use this to select which extending arm
        servoReset = gamepad1.x; //This is to reset the servo down
        servoDown = gamepad1.y; //This is to swing the servo down
        servoUp = gamepad1.b; //This is to swing the servo up

        telemetry.addData("Joy: ", "y:" + drivePower + "x:" + driveBalance + "a:" );

        //Servo priorities
        if (servoReset) {
            servo_state_reset();
        } else if (servoUp) {
            set_servo_drive(true);
        } else if (servoDown) {
            set_servo_drive(false);
        }

        //Arm Priorities
        if ()

        //Lift the arm
        set_arm_drive(driveArmLift, driveArmExtend, 0);
        //Drive the robot
        set_robot_drive(drivePower, driveBalance);
    }

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
    @Override
    public void stop() {
        telemetry.addData("Mode: ", "stop");
        robot_halt();
    }

}