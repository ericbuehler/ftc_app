package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
    final static double MAX_DRIVE_PWR_SOFT = MAX_MOTOR_PWR_SOFT; //We shouldn't drive any faster than this
    final static double MIN_DRIVE_PWR_SOFT = MIN_MOTOR_PWR_SOFT; //We shouldn't drive any faster than this

    //For joystick centering threshold
    float static double MAX_ABS_JOY_ZERO = 0.05; //This value is close enough to zero

    //We want to add a timer here so that we can see how much time we have left
    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();

    //Need to have some variables to control the robot
    DcMotor driverMotor;
    DcMotor passengerMotor;
    DcMotorController driveController;

    final static String driverMotorName = "motor1";
    final static String passengerMotorName = "motor2";
    final static String driveControllerName = "ctl1";

    //For driving motors
    float drivePower = 0.0;
    float driveBalance = 0.0;


//TODO: Need to add telemetry options

    /*
     * This function will read the whole state of the robot layout
     */
    public void robot_read_layout() {
        //Initialize all of objects for robot devices
        //Motors
        driverMotor = hardwareMap.dcMotor.get(driverMotorName);
        passengerMotor = hardwareMap.dcMotor.get(passengerMotorName);
        //Motor Controllers
        driveController = hardwareMap.dcMotorController.get(driveControllerName);
    }

    /*
     * This function will initialize the robot (this is mostly used for arms)
     */
    public void robot_state_reset() {

    }

    /*
     * This function is used for driving the robot
     */
    public void set_robot_drive(float joyPower, float joyBalance) {
        //Set the initial power value
        float driverMotorPwr = joyPower;
        float passengerMotorPwr = joyPower;

        //Now balance out the power
        //If the Balance is 1 then all driver
        //If the Balance is -1 then all passenger
        if (joyBalance.abs() > MAX_ABS_JOY_ZERO) {
            if (joyBalance > 0) { //All driver
                driverMotorPwr = joyBalance * driverMotorPwr;
                passengerMotorPwr = 1 - joyBalance * passengerMotorPwr;
            } else { //All passenger
                driverMotorPwr = 1 + joyBalance * driverMotorPwr;
                passengerMotorPwr = -joyBalance * passengerMotorPwr;
            }
        }
        driverMotor.setPower(driverMotorPwr);
        passengerMotor.setPower(passengerMotorPwr);

    }

    /*
     * This function will stop all robot activity call this in init and stop
     */
    public void robot_halt() {
        //Stop all driving motors
        set_robot_drive(0,0);
        //Stop all arm motors
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        telemetry.addData("Mode: ", "init");
        robot_read_layout();
        robot_halt();
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

        //Set the value to the max allowed
        drivePower *= MAX_DRIVE_PWR_SOFT;
/*
        if (drivePower > MAX_DRIVE_PWR_SOFT) {
            drivePower = MAX_DRIVE_PWR_SOFT;
        } else if (drivePower < MIN_DRIVE_PWR_SOFT) {
            drivePower = MIN_DRIVE_PWR_SOFT;
        }
*/
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