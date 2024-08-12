package org.firstinspires.ftc.teamcode.LaserRoadrunner;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.SparkFunOTOS;

import com.acmerobotics.roadrunner.geometry.Pose2d;

//unused but may potentially have a use


/**
 * This class provides an abstraction layer for the SparkFun OTOS odometry sensor.
 * It simplifies interaction with the sensor by handling initialization,
 * calibration, and data retrieval.
 */
public class RRAbstarctionLayer {
    // Reference to the SparkFunOTOS sensor object
    SparkFunOTOS optical;

    Pose2d currentPoseEstimate;

    /**
     * Constructor for the RRAbstarctionLayer class.
     * Initializes the odometry sensor and performs necessary configuration.
     *
     * @param hardwareMap The hardware map used to access robot hardware
     */


    //Declare the constructor for the class
    public RRAbstarctionLayer(HardwareMap hardwareMap) {

        // Get the SparkFunOTOS sensor from the hardware map
        optical = hardwareMap.get(SparkFunOTOS.class, "sfe_otos");

        // Set the offset location of the sensor relative to the robot center
        SparkFunOTOS.otos_pose2d_t offsetPose = optical.new otos_pose2d_t(3.7, -.55, 0);
        optical.setOffset(offsetPose);

        // Set scalar correction factors for translation and rotation
        optical.setLinearScalar(1.0);
        optical.setAngularScalar(1.0);

        // Calibrate IMU. Must be stationary and flat during this time! By default, this will take
        // about 612ms, but can be made faster by taking fewer samples
        optical.calibrateImu();

        // Reset tracking algorithm on OTOS
        optical.resetTracking();

        // Set known location on field
        SparkFunOTOS.otos_pose2d_t newPose = optical.new otos_pose2d_t(50, 50, 0);
        optical.setPosition(newPose);



        /**
         * Updates the internal pose estimate based on the latest sensor data.
         * This method might not be used in all use cases.
         */
    }

    /**
     * Getter method to retrieve the X,Y,Z coordinate (position) from the sensor data.
     *
     * @return The X,Y,Z coordinate (inches/degrees) from the odometry sensor
     */    public double getX() {
        SparkFunOTOS.otos_pose2d_t pose = optical.getPosition();
        return pose.x;
    }

    public double getY() {
        SparkFunOTOS.otos_pose2d_t pose = optical.getPosition();
        return pose.y;
    }

    public double getZ() { // Assuming z represents heading (angle)
        SparkFunOTOS.otos_pose2d_t pose = optical.getPosition();
        return Math.toRadians(pose.h); // Convert to radians if needed
    }


    // Update pose estimate (if applicable)
    public void updatePoseEstimateRR() {
        currentPoseEstimate = new Pose2d(getX(), getY(), getZ());
    }

    public Pose2d getUpdatedPOSE() {
    updatePoseEstimateRR();

        return currentPoseEstimate;
    }

    public void calibrateImu() {
        optical.calibrateImu();
    }

    public void resetTracking() {
        optical.resetTracking();
        // SparkFunOTOS.otos_pose2d_t newPose = myOdometrySensor.new otos_pose2d_t(5, 10, 45);
    }

   /* public PoseVelocity2d updatePoseEstimate() {
        SparkFunOTOS.otos_pose2d_t opticalPose = optical.getPosition();
        Pose2d pose = new Pose2d(opticalPose.x, opticalPose.y, opticalPose.h);

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }*/

}
