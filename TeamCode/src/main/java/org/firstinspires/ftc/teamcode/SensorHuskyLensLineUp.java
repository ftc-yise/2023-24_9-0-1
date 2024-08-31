package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.yise.DriveColorExample;
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LedLights;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.RoadRunnerDriving;

@TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
public class SensorHuskyLensLineUp extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    @Override
    public void runOpMode() {
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");

        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFrontDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBackDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "RightBackDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFrontDrive");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // create instance of drive class
        RoadRunnerDriving rrDrive = new RoadRunnerDriving(hardwareMap);
        // create instance of lift arm class
        LiftArm arm = new LiftArm(hardwareMap);
        // create instance of intake system class
        IntakeSystem intakeSystem = new IntakeSystem(hardwareMap);

        LedLights leds = new LedLights(hardwareMap);

        Parameters parameters = new Parameters();

        DriveColorExample colorSensors = new DriveColorExample(hardwareMap);




        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        if (Parameters.allianceColor == Parameters.Color.RED) {
            leds.setLed(LedLights.ledStates.RED);
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            leds.setLed(LedLights.ledStates.BLUE);
        }
        telemetry.update();
        waitForStart();


        // Initialize x as a default value
        int x1 = 0;
        int x2 = 0;
        int x3 = 0;
        int x4 = 0;

        double drive = 0;
        double legnth = 0;

        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                rrDrive.updateFromDpad(-0.2, 0, 0);
            } else if (gamepad1.dpad_up) {
                rrDrive.updateFromDpad(0.2, 0, 0);
            } else if (gamepad1.dpad_left) {
                rrDrive.updateFromDpad(0, 0.2, 0);
            } else if (gamepad1.dpad_right) {
                rrDrive.updateFromDpad(0, -0.2, 0);
            } else {
                rrDrive.updateMotorsFromStick(gamepad1);
            }
            rrDrive.update();

            // Get blocks from HuskyLens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            // Update the telemetry with blocks' details
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            telemetry.update();

            // Update x based on blocks' information if blocks are not empty

            if (blocks.length == 1) {
                x1 = blocks[0].x; // x coordinate of the first block
                telemetry.addLine("1");
                legnth = 1;

            } else if (blocks.length == 2) {
                x1 = blocks[0].x; // x coordinate of the first block
                x2 = blocks[1].x; // x coordinate of the second block
                telemetry.addLine("2");
                legnth = 2;
            } else if (blocks.length == 3) {
                x1 = blocks[0].x; // x coordinate of the first block
                x2 = blocks[1].x; // x coordinate of the second block
                x3 = blocks[2].x; // x coordinate of the third block
                telemetry.addLine("3");
                legnth = 3;
            } else if (blocks.length == 4) {
                x1 = blocks[0].x; // x coordinate of the first block
                x2 = blocks[1].x; // x coordinate of the second block
                x3 = blocks[2].x; // x coordinate of the third block
                x4 = blocks[3].x; // x coordinate of the forth block
                telemetry.addLine("4");
                legnth = 4;
            }
            if (blocks.length < 1) {
                telemetry.addLine("Whoops: No Tag Detected!");
                leds.setLed(LedLights.ledStates.CELEBRATION);
            }
            else if (blocks.length == 1) {
                if (x1 != -1) { // Check if x has been updated
                    if (x1 < 140) {
                        rrDrive.updateFromDpad(0, 0, 0);
                    } else if (x1 > 160) {
                        rrDrive.updateFromDpad(0, -0, 0);
                    }
                }
            }
            else if (blocks.length > 1) {
                double AverageOfX = x1 + x2 + x3 + x4;
                telemetry.addData("average", AverageOfX / legnth);

                if ((AverageOfX / blocks.length) < 140) {
                    drive = 1;
                } else if ((AverageOfX / blocks.length) > 160) {
                    drive = 2;
                } else {
                    drive = 0;
                }
            }

            // Control logic based on x value and right trigger
            if (gamepad1.right_trigger > 0.75) {
                if (drive == 2){
                    leftRear.setPower(-0.7);
                    leftFront.setPower(0.7);
                    rightFront.setPower(-0.7);
                    rightRear.setPower(0.7);
                } else if (drive == 1) {
                    leftRear.setPower(0.7);
                    leftFront.setPower(-0.7);
                    rightFront.setPower(0.7);
                    rightRear.setPower(-0.7);
                } else {
                    leftRear.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }
            }
            }
        }
    }