package org.firstinspires.ftc.teamcode.demobotdrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Beeline", group="Linear Opmode")
public class BeelineDrive extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public float speedmulti = .5f;
    public float Yout = 0;

    private Servo ser;
    //private CRServo barrel = null;
    TouchSensor limit;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        ser = hardwareMap.get(Servo.class, "ser");
        //barrel = hardwareMap.get(CRServo.class, "barrel" );

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        ser.setDirection(Servo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        /*if (limit.isPressed()) {
            leftFrontDrive.setPower(0);
        } else { // Otherwise, run the motor
            leftFrontDrive.setPower(1);
        }*/


        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("motor Power:", leftBackDrive.getPower());


            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value
            double turn     =  .5 * gamepad1.left_stick_y;
            double vertical     =  .5 * gamepad1.left_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftPower  = vertical  - turn;
            double rightPower = vertical  + turn;

            // Speed Boost (Adjust value as needed)
            if (gamepad1.a) {
                leftPower *= 1.5f;
                rightPower *= 1.5f;
            }

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));


            if (gamepad1.left_bumper) {
                ser.setPosition(Servo.MIN_POSITION);
            } else if (gamepad1.right_bumper) {
                ser.setPosition(Servo.MAX_POSITION);
            }

            if (max > 1.0) {
                leftPower  /= max;
                rightPower /= max;
            }
            leftBackDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
            leftFrontDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", rightPower);
            telemetry.addData("Back  left/Right", rightPower);
            //telemetry.addData("limitSwitchState", limitSwitchState);


            //telemetry.addData("CR positon",fireServo.getPosition());
            //telemetry.addData("CR class",fireServo.getClass());
            telemetry.update();
        }
    }}


