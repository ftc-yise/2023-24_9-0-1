package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.RRAbstarctionLayer;
@TeleOp(name="Throttles", group="Linear Opmode")
public class Triggered extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();



    boolean canToggleSlowMode = true;
    double speedMultiplier = 1;

    @Override
    public void runOpMode() {
        RRAbstarctionLayer laser = new RRAbstarctionLayer(hardwareMap);

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD

        );


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double theta = laser.getZ() + Math.PI;
            double x = gamepad2.left_trigger;
            double y = gamepad2.right_trigger;
            double heading = 0;

            if (gamepad1.dpad_left){
                x = -1 * speedMultiplier;
                //heading = -0.018;
            } else if (gamepad1.dpad_right){
                x = 1 * speedMultiplier;
                //heading = 0.018;
            } else if (gamepad1.dpad_up){
                y = -1 * speedMultiplier;
            } else if (gamepad1.dpad_down){
                y = 1 * speedMultiplier;
            }


            double xOut = (x * Math.cos(theta)) - (y * Math.sin(theta));
            double yOut = (y * Math.cos(theta)) + (x * Math.sin(theta));

            double leftFrontPower  = x - heading;
            double rightFrontPower = y + heading;
            double leftBackPower   = x - heading;
            double rightBackPower  = y + heading;

           /*double leftFrontPower  = yOut - xOut + heading;
            double rightFrontPower = yOut + xOut - heading;
            double leftBackPower   = yOut + xOut + heading;
            double rightBackPower  = yOut - xOut - heading;*/

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            /**
             * Slow mode toggle
             */
            //If input released, slow mode can be toggled again. This prevents an infinite loop of toggling.
            if (!gamepad1.y) {
                canToggleSlowMode = true;
            }

            if (gamepad1.y && canToggleSlowMode) {
                canToggleSlowMode = false;
                speedMultiplier = 0.5;
            }
            laser.getUpdatedPOSE();
            telemetry.addData("z", laser.getZ());

            telemetry.addData("yout", yOut);
            telemetry.addData("xout", xOut);

            telemetry.addData("y", y);
            telemetry.addData("x", x);

            telemetry.addData("heading", heading);

            telemetry.update();
        }
    }
}


