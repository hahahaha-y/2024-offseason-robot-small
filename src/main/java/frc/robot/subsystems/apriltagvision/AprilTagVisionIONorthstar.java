// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.Alert;

import java.util.function.Supplier;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.cameraIds;
import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.instanceNames;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
    private static final int cameraResolutionWidth = 1280;
    private static final int cameraResolutionHeight = 720;
    private static final int cameraAutoExposure = 1;
    private static final int cameraExposure = 150;
    private static final int cameraGain = 60;
    private static final int fps = 120;
    private static final int brightness = 50;
    private static final int contrast = 60;
    private static final int buffersize = 1;
    private static final double disconnectedTimeout = 0.5;
    private final Supplier<frc.robot.FieldConstants.AprilTagLayoutType> aprilTagTypeSupplier;
    private final StringPublisher tagLayoutPublisher;
    private final DoubleArraySubscriber observationSubscriber;
    private final DoubleArraySubscriber demoObservationSubscriber;
    private final IntegerSubscriber fpsSubscriber;
    private final Alert disconnectedAlert;
    private final Timer disconnectedTimer = new Timer();
    private frc.robot.FieldConstants.AprilTagLayoutType lastAprilTagType = null;

    public AprilTagVisionIONorthstar(Supplier<frc.robot.FieldConstants.AprilTagLayoutType> aprilTagTypeSupplier, int index) {
        this.aprilTagTypeSupplier = aprilTagTypeSupplier;
        var northstarTable = NetworkTableInstance.getDefault().getTable(instanceNames[index]);
        var configTable = northstarTable.getSubTable("config");
        configTable.getStringTopic("camera_id").publish().set(cameraIds[index]);
        configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
        configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
        configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
        configTable.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
        configTable.getIntegerTopic("camera_gain").publish().set(cameraGain);
        configTable.getIntegerTopic("fps").publish().set(fps);
        configTable.getIntegerTopic("brightness").publish().set(brightness);
        configTable.getIntegerTopic("contrast").publish().set(contrast);
        configTable.getIntegerTopic("buffersize").publish().set(buffersize);
        configTable.getDoubleTopic("fiducial_size_m").publish().set(frc.robot.FieldConstants.aprilTagWidth);
        tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();

        var outputTable = northstarTable.getSubTable("output");
        observationSubscriber =
                outputTable
                        .getDoubleArrayTopic("observations")
                        .subscribe(
                                new double[]{}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        demoObservationSubscriber =
                outputTable
                        .getDoubleArrayTopic("demo_observations")
                        .subscribe(
                                new double[]{}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0);

        disconnectedAlert =
                new Alert("No data from \"" + instanceNames[index] + "\"", Alert.AlertType.ERROR);
        disconnectedTimer.start();
    }

    public void updateInputs(AprilTagVisionIOInputs inputs) {
        // Publish tag layout
        var aprilTagType = aprilTagTypeSupplier.get();
        if (aprilTagType != lastAprilTagType) {
            lastAprilTagType = aprilTagType;
            tagLayoutPublisher.set(aprilTagType.getLayoutString());
        }

        // Get observations
        var queue = observationSubscriber.readQueue();
        inputs.timestamps = new double[queue.length];
        inputs.frames = new double[queue.length][];
        for (int i = 0; i < queue.length; i++) {
            inputs.timestamps[i] = queue[i].timestamp / 1000000.0;
            inputs.frames[i] = queue[i].value;
        }
        inputs.demoFrame = new double[]{};
        for (double[] demoFrame : demoObservationSubscriber.readQueueValues()) {
            inputs.demoFrame = demoFrame;
        }
        inputs.fps = fpsSubscriber.get();

        // Update disconnected alert
        if (queue.length > 0) {
            disconnectedTimer.reset();
        }
        disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
    }
}
