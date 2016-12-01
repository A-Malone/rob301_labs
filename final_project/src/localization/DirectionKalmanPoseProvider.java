package localization;

import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Pose;
import lejos.utility.KalmanFilter;
import lejos.utility.Matrix;

public class DirectionKalmanPoseProvider implements PoseProvider, MoveListener, SampleProvider {
	private SampleProvider direction_finder;
	private KalmanFilter kalman_filter;

	// The current estimate of the state
	private float x = 0, y = 0, heading = 0;

	// The values at the last measurement
	private float odo_angle0, distance0;

	// The heading and sensor reading at the start of the move
	private float start_heading;
	private float start_sensor_angle;

	MoveProvider mp;
	boolean current = true;

	public DirectionKalmanPoseProvider(MoveProvider mp, SampleProvider direction_finder) {
		this.mp = mp;
		mp.addMoveListener(this);

		// Setup gyro for readings
		this.direction_finder = direction_finder;

		// Do a quick reading to work out the kinks
		get_direction_reading();

		// Setup the Kalman filter
		Matrix A = Matrix.identity(3, 3);
		// Roll the angles into the control input
		Matrix B = Matrix.identity(3, 3);
		Matrix Q = new Matrix(new double[][] { { 0.1, 0, 0 }, { 0, 0.1, 0 }, { 0, 0, 0.1 } });

		// 2 Sensors
		Matrix C = new Matrix(new double[][] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 1, 0, 0 } });
		Matrix R = new Matrix(new double[][] { { 0.1, 0, 0, 0 }, { 0, 0.1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } });

		kalman_filter = new KalmanFilter(A, B, C, Q, R);
	}

	private float get_direction_reading() {
		float[] sample = new float[direction_finder.sampleSize()];
		direction_finder.fetchSample(sample, 0);
		return sample[0];
	}

	public synchronized Pose getPose() {
		if (!current) {
			updatePose(mp.getMovement());
		}
		return new Pose(x, y, heading);
	}

	public synchronized void moveStarted(Move move, MoveProvider mp) {
		// Initialize the starting heading
		start_heading = heading;
		start_sensor_angle = get_direction_reading();

		odo_angle0 = 0;
		distance0 = 0;
		current = false;
		this.mp = mp;
	}

	public void moveStopped(Move move, MoveProvider mp) {
		updatePose(move);
	}

	private synchronized void updatePose(Move event) {
		// ODOMETRY
		float odo_angle_delta = event.getAngleTurned() - odo_angle0;

		// DIRECTION SENSOR
		// Calculate angle relative to the start of the move in order to provide
		// consistent readings
		float sensor_angle = get_direction_reading();
		float sensor_relative_angle = sensor_angle - start_sensor_angle;

		// From OdometryPoseProvider
		// ------------------------------------------------------------
		float distance = event.getDistanceTraveled() - distance0;
		double dx = 0, dy = 0;
		double headingRad = (Math.toRadians(heading));

		if (event.getMoveType() == Move.MoveType.TRAVEL || Math.abs(odo_angle_delta) < 0.2f) {
			dx = (distance) * (float) Math.cos(headingRad);
			dy = (distance) * (float) Math.sin(headingRad);
		} else if (event.getMoveType() == Move.MoveType.ARC) {
			double turnRad = Math.toRadians(odo_angle_delta);
			double radius = distance / turnRad;
			dy = radius * (Math.cos(headingRad) - Math.cos(headingRad + turnRad));
			dx = radius * (Math.sin(headingRad + turnRad) - Math.sin(headingRad));
		}
		// ------------------------------------------------------------

		// Update the Kalman filter
		Matrix control = new Matrix(new double[][] { { dx }, { dy }, { odo_angle_delta } });
		Matrix measurement = new Matrix(new double[][] { { x + dx }, { y + dy }, { heading + odo_angle_delta },
				{ start_heading + sensor_relative_angle } });
		kalman_filter.update(control, measurement);

		// Update our local cache of the current Kalman filter estimate
		Matrix estimate = kalman_filter.getMean();
		x = (float) estimate.get(0, 0);
		y = (float) estimate.get(1, 0);
		heading = (float) estimate.get(2, 0);

		odo_angle0 = event.getAngleTurned();
		distance0 = event.getDistanceTraveled();
		current = !event.isMoving();
	}

	private float normalize(float angle) {
		float a = angle;
		while (a > 180)
			a -= 360;
		while (a < -180)
			a += 360;
		return a;
	}

	private void setPosition(Point p) {
		x = p.x;
		y = p.y;

		Matrix covariance = kalman_filter.getCovariance();
		covariance.set(0, 0, 0);
		covariance.set(1, 1, 0);

		Matrix mean = kalman_filter.getMean();
		mean.set(0, 0, x);
		mean.set(1, 0, y);

		kalman_filter.setState(mean, covariance);

		current = true;
	}

	private void setHeading(float heading) {
		this.heading = heading;

		Matrix covariance = kalman_filter.getCovariance();
		covariance.set(2, 2, 0);

		Matrix mean = kalman_filter.getMean();
		mean.set(2, 0, heading);

		kalman_filter.setState(mean, covariance);

		current = true;
	}

	@Override
	public int sampleSize() {
		return 3;
	}

	@Override
	public void fetchSample(float[] sample, int offset) {
		if (!current) {
			updatePose(mp.getMovement());
		}
		sample[offset + 0] = x;
		sample[offset + 1] = y;
		sample[offset + 2] = heading;
	}

	@Override
	public void setPose(Pose aPose) {
		setPosition(aPose.getLocation());
		setHeading(aPose.getHeading());
	}
}
