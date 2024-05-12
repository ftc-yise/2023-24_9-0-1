package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.DriveColorExample;
import org.firstinspires.ftc.teamcode.yise.IntakeSystem;
import org.firstinspires.ftc.teamcode.yise.LedLights;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.PoseStorage;
import org.firstinspires.ftc.teamcode.yise.TensorflowVision;

import org.firstinspires.ftc.teamcode.LaserRoadrunner.RRAbstarctionLayer;

import org.firstinspires.ftc.teamcode.yise.Parameters;


@Autonomous(name="Autonomous", group="Linear Opmode")
public class Auto extends LinearOpMode {
    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();
    int prop;

    public enum Prop {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public Prop location;

    public void propDetection(TensorflowVision vision) {
        prop = vision.getPropPosition();
        if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            if (prop == 2) {
                location = Prop.RIGHT;
            } else if (prop == 0) {
                location = Prop.MIDDLE;
            } else if (prop == 1) {
                location = Prop.LEFT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            if (prop == 0) {
                location = Prop.RIGHT;
            } else if (prop == 1) {
                location = Prop.MIDDLE;
            } else if (prop == 2) {
                location = Prop.LEFT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            if (prop == 2) {
                location = Prop.LEFT;
            } else if (prop == 1) {
                location = Prop.MIDDLE;
            } else if (prop == 0) {
                location = Prop.RIGHT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            if (prop == 1) {
                location = Prop.LEFT;
            } else if (prop == 0) {
                location = Prop.MIDDLE;
            } else if (prop == 2) {
                location = Prop.RIGHT;
            }
        }
    }
    
    //Drop purple pixel and navigate to white stack
    public TrajectorySequence purplePixel(SampleMecanumDrive drive, Pose2d startPose, LiftArm arm) {
        //Create all position variables that will be changed
        double heading = 0;
        double x = 0;
        double y = 0;


        //Get the alliance color and starting side of truss
        //Calculate coordinates depending on prop location
        if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR){
            heading = -90;
            if (location == Prop.RIGHT) {
                y = -36.0;
                x = 19.5;
            } else if (location == Prop.MIDDLE) {
                y = -32.5;
                x = 12.0;
            } else if (location == Prop.LEFT) {
                y = -35.0;
                heading = 0;
                x = 7.5;
            }
        } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            heading = -90;
            if (location == Prop.RIGHT) {
                y =-32;
                heading = 180;
                x = -32.5;
            } else if (location == Prop.MIDDLE) {
                y = -32;
                x = -42;
            } else if (location == Prop.LEFT) {
                y = -37;
                x = -49.5;
            }
        } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR){
            heading = 90;
            if (location == Prop.LEFT) {
                y = 36.0;
                x = 25.5;
            } else if (location == Prop.MIDDLE) {
                y = 32.5;
                x = 12.0;
            } else if (location == Prop.RIGHT) {
                y = 32.0;
                heading = 0;
                x = 9;
            }
        } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR){
            heading = 90;
            if (location == Prop.LEFT) {
                y = 34;
                heading = 180;
                x = -32.5;
            } else if (location == Prop.MIDDLE) {
                y = 33;
                x = -38;
            } else if (location == Prop.RIGHT) {
                y = 38.0;
                x = -43.5;
            }
        }

        //Build the trajectory sequence
        TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
                //Wait for however long drivers want before moving
                .waitSeconds(Parameters.WAIT)
                //Go to calculated position
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
                .build();

        //Return the built sequence so it can be run
        return mySequence;
    }

    //Navigating to and dropping yellow pixel
    public TrajectorySequence middleLane(SampleMecanumDrive drive, Pose2d startPose, LiftArm arm) {

        TrajectorySequence redAnnoyingPixel = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .forward(3)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(180))
                .forward(20)
                .build();

        TrajectorySequence redToMiddle = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .lineToLinearHeading(new Pose2d(-56, -12, Math.toRadians(180)))
                //.forward(3)
                .build();

        TrajectorySequence blueAnnoyingPixel = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .forward(3)
                .splineTo(new Vector2d(-40, 9), Math.toRadians(180))
                .forward(16)
                .build();

        TrajectorySequence blueToMiddle = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .lineToLinearHeading(new Pose2d(-56, 9, Math.toRadians(180)))
                //.forward(3)
                .build();



        if (Parameters.allianceColor == Parameters.Color.RED) {
            if (location == Prop.LEFT) {
                return redAnnoyingPixel;
            } else {
                return redToMiddle;
            }
        } else {
            if (location == Prop.RIGHT) {
                return blueAnnoyingPixel;
            } else {
                return blueToMiddle;
            }
        }
    }

    //Navigating to and dropping yellow pixel
    public TrajectorySequence yellowPixel(SampleMecanumDrive drive, Pose2d startPose, LiftArm arm, IntakeSystem intake) {
        //Position of backboard, y will change based on prop, x is constant distance from it
        double backdropX = 47;
        double backdropY = 0;
        double backdropHeading = 180;
        double drivingY = 0;

        if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            drivingY = -10;
            if (location == Prop.RIGHT) {
                backdropY = -42.0;
            } else if (location == Prop.MIDDLE) {
                backdropY = -35.0;
            } else if (location == Prop.LEFT) {
                backdropY = -29.0;
            }
        } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            drivingY = -10;
            if (location == Prop.LEFT) {
                backdropY = -28;
            } else if (location == Prop.MIDDLE) {
                backdropY = -33;
            } else if (location == Prop.RIGHT) {
                backdropY = -39;
            }
        } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            drivingY = 8;
            if (location == Prop.LEFT) {
                backdropY = 42.0;
            } else if (location == Prop.MIDDLE) {
                backdropY = 35.0;
            } else if (location == Prop.RIGHT) {
                backdropY = 29.0;
            }
        } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            drivingY = 8;
            if (location == Prop.RIGHT) {
                backdropY = 23;
            } else if (location == Prop.MIDDLE) {
                backdropY = 27;
            } else if (location == Prop.LEFT) {
                backdropY = 35;
            }
        }
        TrajectorySequence backdrop = drive.trajectorySequenceBuilder(startPose)
                .forward(3)
                .addDisplacementMarker( () -> {
                    arm.extend(LiftArm.Distance.AUTO);
                })
                .lineToLinearHeading(new Pose2d(backdropX, backdropY, Math.toRadians(backdropHeading)))
                .back(5)
                .build();

        TrajectorySequence farDriveToBackdrop = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(28, drivingY, Math.toRadians(180)))
                /*.addDisplacementMarker(10, () -> {
                    intake.runIntakeSystem(-0.5);
                })*/
                .addDisplacementMarker(40, () -> {
                    arm.extend(LiftArm.Distance.AUTO);
                })
                .splineTo(new Vector2d(backdropX, backdropY), Math.toRadians(0))
                .back(5)
                .build();

        if (Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            return backdrop;
        } else {
            return farDriveToBackdrop;
        }
    }

    public TrajectorySequence parking(SampleMecanumDrive drive, Pose2d startPose) {
        double x = 48.0;
        double y = 0.0;
        double heading = 180.0;

        if (Parameters.allianceColor == Parameters.Color.RED) {
            if (Parameters.endingPosition == Parameters.EndingPosition.LEFT) {
                y = -11;
            } else if (Parameters.endingPosition == Parameters.EndingPosition.CENTER) {
                y = -35.0;
            } else if (Parameters.endingPosition == Parameters.EndingPosition.RIGHT) {
                y = -50;
            }
        } else {
            if (Parameters.endingPosition == Parameters.EndingPosition.LEFT) {
                y = 50;
            } else if (Parameters.endingPosition == Parameters.EndingPosition.CENTER) {
                y = 35.0;
            } else if (Parameters.endingPosition == Parameters.EndingPosition.RIGHT) {
                y = 11;
            }
        }


        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .back(-5)
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(heading)))
                .build();
        return park;
    }

    public TrajectorySequence driveToWhiteStack(SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence seqRed = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .splineTo(new Vector2d(5, -14), Math.toRadians(180))
                .forward(40)
                .splineTo(new Vector2d(-58, -22), Math.toRadians(-160))
                .build();

        TrajectorySequence seqBlue = drive.trajectorySequenceBuilder(startPose)
                .setReversed(false)
                .splineTo(new Vector2d(5, 7), Math.toRadians(180))
                .forward(40)
                .splineTo(new Vector2d(-56, 17), Math.toRadians(160))
                .build();
        if (Parameters.allianceColor == Parameters.Color.RED) {
            return seqRed;
        } else {
            return seqBlue;
        }
    }

    public TrajectorySequence driveToBackdrop(SampleMecanumDrive drive, Pose2d startPose, LiftArm arm) {
        double backdropX = 40;
        double splineY = 0;
        double backdropY = 0;

        if (Parameters.allianceColor == Parameters.Color.RED) {
            backdropY = -36;
            splineY = -13;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            splineY = 6;
            backdropY = 27;
        }

        TrajectorySequence seq = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-28, splineY), Math.toRadians(0))
                .back(45)
                .splineTo(new Vector2d(backdropX, backdropY), Math.toRadians(0))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(48, backdropY, Math.toRadians(180)))
                .addDisplacementMarker(50, () -> {
                    arm.extend(LiftArm.Distance.LOW);
                })

                .build();

        return seq;
    }

    @Override
    public void runOpMode() {

        //Initialize RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LiftArm arm = new LiftArm(hardwareMap);
        TensorflowVision vision = new TensorflowVision(hardwareMap);
        LedLights leds = new LedLights(hardwareMap);
        DriveColorExample colorSensors = new DriveColorExample(hardwareMap);
        IntakeSystem intake = new IntakeSystem(hardwareMap);

        RRAbstarctionLayer laser = new RRAbstarctionLayer(hardwareMap, drive);

        leds.setLed(LedLights.ledStates.DARK);
        //Sense cones

        while (!isStarted()) {
            propDetection(vision);

            telemetry.addData("Prop: ", location);
            telemetry.addData("Side: ", Parameters.autoConfig);
            telemetry.addData("Alliance: ", Parameters.allianceColor);
            telemetry.addData("Park pos: ", Parameters.endingPosition);
            telemetry.addData("Beginning delay: ", Parameters.WAIT);

            leds.setLed(LedLights.ledStates.DARK);
            telemetry.update();

        }

        if (isStopRequested()) return;


        /**
         * Set starting position for robot
         */
        double startX;
        double startY;
        double startHeading;

        if (Parameters.allianceColor == Parameters.Color.BLUE){
            startY = 62;
            startHeading = 90;
            leds.setLed(LedLights.ledStates.BLUE);
        } else {
            startY = -62;
            startHeading = -90;
            leds.setLed(LedLights.ledStates.RED);
        }

        if (Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR){
            startX = 15.4375;
        } else {
            startX = -39.4375;
        }

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeading));
        drive.setPoseEstimate(startPose);

        /**
         * Build trajectories
         */

        //Dropping purple pixel
        TrajectorySequence dropPurplePixel = purplePixel(drive, startPose, arm);
        drive.followTrajectorySequence(dropPurplePixel);
        arm.dropPurplePixel();

        //Driving to backdrop/getting white stack first
        if (Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            //Drive to white stack, drive to backdrop, park
            TrajectorySequence driveToMiddleLane = middleLane(drive, dropPurplePixel.end(), arm);
            TrajectorySequence driveToBackdrop = yellowPixel(drive, driveToMiddleLane.end(), arm, intake);
            TrajectorySequence pickupWhitePixels = driveToWhiteStack(drive, driveToBackdrop.end());
            TrajectorySequence returnToBackdrop = driveToBackdrop(drive, pickupWhitePixels.end(), arm);

            drive.followTrajectorySequence(driveToMiddleLane);

            //Intake
            /*int intakeLoops = 0;
            while (colorSensors.getBackPixelColor() == DriveColorExample.Colors.NONE || colorSensors.getFrontPixelColor() == DriveColorExample.Colors.NONE) {
                if (intakeLoops >= 5) {
                    break;
                } else {
                intakeLoops++;
                intake.runIntakeSystem(0.75);
                sleep(400);
                intake.runIntakeSystem(-0.25);
                sleep(100);
                }
            }*/

            drive.followTrajectorySequence(driveToBackdrop);
            intake.runIntakeSystem(0);
            //Open trapdoor
            arm.openTrapdoor();
            sleep(500);
            arm.retract();
            arm.closeTrapdoor();
            intake.runIntakeSystem(0.75);

            drive.followTrajectorySequence(pickupWhitePixels);

            int intakeLoops = 0;
            while (colorSensors.getBackPixelColor() == DriveColorExample.Colors.NONE || colorSensors.getFrontPixelColor() == DriveColorExample.Colors.NONE) {
                if (intakeLoops >= 3) {
                    break;
                } else {
                    intakeLoops++;
                    intake.runIntakeSystem(0.75);
                    sleep(400);
                    intake.runIntakeSystem(-0.25);
                    sleep(100);
                }
            }
            intake.runIntakeSystem(-0.5);

            drive.followTrajectorySequence(returnToBackdrop);
            intake.runIntakeSystem(0);

            //Open trapdoor
            arm.openTrapdoor();
            sleep(500);
            arm.retract();
            arm.closeTrapdoor();

            sleep(5000);


        } else {

            TrajectorySequence driveToBackdrop = yellowPixel(drive, dropPurplePixel.end(), arm, intake);
            TrajectorySequence park = parking(drive, driveToBackdrop.end());

            drive.followTrajectorySequence(driveToBackdrop);
            //Open trapdoor
            arm.openTrapdoor();
            sleep(1000);
            arm.retract();
            arm.closeTrapdoor();

            drive.followTrajectorySequence(park);
        }

        //PoseStorage.currentPose = drive.getPoseEstimate();
        PoseStorage.currentPose = laser.getUpdatedPOSE();
    }
}
