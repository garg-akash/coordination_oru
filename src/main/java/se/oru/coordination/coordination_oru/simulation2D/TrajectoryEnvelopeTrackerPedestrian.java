package se.oru.coordination.coordination_oru.simulation2D;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Scanner;
import java.util.TreeMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;

public abstract class TrajectoryEnvelopeTrackerPedestrian extends AbstractTrajectoryEnvelopeTracker implements Runnable {

	protected static final long WAIT_AMOUNT_AT_END = 3000;
	protected static final double EPSILON = 0.01;
	protected double overallDistance = 0.0;
	protected double totalDistance = 0.0;
	protected double elapsedTrackingTime = 0.0;
	private Thread th = null;
	protected State state = null;
	private int maxDelayInMilis = 0;
	private Random rand = new Random();
	
	/**
	 * Read a path from a file.
	 * @param fileName The name of the file containing the pedestrian path
	 * @return The pedestrian path read from the file
	 */
	public static Trajectory loadPedestrianTrajectoryFromFile(String fileName) {
		ArrayList<PoseSteering> ret = new ArrayList<PoseSteering>();
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if (line.length() != 0) {
					String[] oneline = line.split(" |\t");
					PoseSteering ps = null;
					ps = new PoseSteering(
					     new Double(oneline[0]).doubleValue(),
						 new Double(oneline[1]).doubleValue(),
						 new Double(oneline[2]).doubleValue(),
						 0.0);					
					}
					ret.add(ps);
			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }
		
		// FIX return proper trajectory.
		return new Trajectory();
	}
	
	public TrajectoryEnvelopeTrackerPedestrian(TrajectoryEnvelope te, int timeStep, double temporalResolution, TrajectoryEnvelopeCoordinator tec, TrackingCallback cb) {
		super(te, temporalResolution, tec, timeStep, cb);
		this.state = new State(0.0, 0.0);
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
		this.overallDistance = totalDistance;
		this.th = new Thread(this, "Pedestrian tracker " + te.getComponent());
		this.th.setPriority(Thread.MAX_PRIORITY);
	}
	
	@Override
	public void onTrajectoryEnvelopeUpdate(TrajectoryEnvelope te) {
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
		this.overallDistance = totalDistance;
	}
	
	@Override
	public void startTracking() {
		while (this.th == null) {
			try { Thread.sleep(10); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		this.th.start();
	}

	public static double computeDistance(Trajectory traj, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < endIndex; i++) {
			ret += traj.getPose()[i].distanceTo(traj.getPose()[i+1]);
		}
		return ret;
	}

	private double computeDistance(int startIndex, int endIndex) {
		return computeDistance(this.traj, startIndex, endIndex);
	}
		
	// TODO: Modify
	public void integratePedestrianState(double deltaTime) {
		synchronized(state) {
			Derivative a = Derivative.evaluate(state, time, 0.0, new Derivative(), slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
			Derivative b = Derivative.evaluate(state, time, deltaTime/2.0, a, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
			Derivative c = Derivative.evaluate(state, time, deltaTime/2.0, b, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR,MAX_ACCELERATION);
			Derivative d = Derivative.evaluate(state, time, deltaTime, c, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
	
			double dxdt = (1.0f / 6.0f) * ( a.getVelocity() + 2.0f*(b.getVelocity() + c.getVelocity()) + d.getVelocity() ); 
		    double dvdt = (1.0f / 6.0f) * ( a.getAcceleration() + 2.0f*(b.getAcceleration() + c.getAcceleration()) + d.getAcceleration() );
			
		    state.setPosition(state.getPosition()+dxdt*deltaTime);
		    state.setVelocity(state.getVelocity()+dvdt*deltaTime);
		}
	}
			
	@Override
	public void setCriticalPoint(int criticalPointToSet) {
		
		if (this.criticalPoint != criticalPointToSet) {
			
			//A new intermediate index to stop at has been given
			if (criticalPointToSet != -1 && criticalPointToSet > getRobotReport().getPathIndex()) {			
				//Store backups in case we are too late for critical point
				double totalDistanceBKP = this.totalDistance;
				int criticalPointBKP = this.criticalPoint;
	
				this.criticalPoint = criticalPointToSet;
				//TOTDIST: ---(state.getPosition)--->x--(computeDist)--->CP
				this.totalDistance = computeDistance(0, criticalPointToSet);
			}

			//Critical point <= current position, ignore -- WHY??
			else if (criticalPointToSet != -1 && criticalPointToSet <= getRobotReport().getPathIndex()) {
				metaCSPLogger.warning("Ignored critical point (" + te.getComponent() + "): " + criticalPointToSet + " because robot is already at " + getRobotReport().getPathIndex() + " (and current CP is " + this.criticalPoint + ")");
			}
			
			//The critical point has been reset, go to the end
			else if (criticalPointToSet == -1) {
				this.criticalPoint = criticalPointToSet;
				this.totalDistance = computeDistance(0, traj.getPose().length-1);
				metaCSPLogger.finest("Set critical point (" + te.getComponent() + "): " + criticalPointToSet);
			}
		}
		
		//Same critical point was already set
		else {
			metaCSPLogger.warning("Critical point (" + te.getComponent() + ") " + criticalPointToSet + " was already set!");
		}
		
	}
	
	@Override
	public RobotReport getRobotReport() {
		if (state == null) return null;
		if (!this.th.isAlive()) return new RobotReport(te.getRobotID(), traj.getPose()[0], -1, 0.0, 0.0, -1);
		synchronized(state) {
			Pose pose = null;
			int currentPathIndex = -1;
			double accumulatedDist = 0.0;
			Pose[] poses = traj.getPose();
			for (int i = 0; i < poses.length-1; i++) {
				double deltaS = poses[i].distanceTo(poses[i+1]);
				accumulatedDist += deltaS;
				if (accumulatedDist > state.getPosition()) {
					double ratio = 1.0-(accumulatedDist-state.getPosition())/deltaS;
					pose = poses[i].interpolate(poses[i+1], ratio);
					currentPathIndex = i;
					break;
				}
			}
			if (currentPathIndex == -1) {
				currentPathIndex = poses.length-1;
				pose = poses[currentPathIndex];
			}
			return new RobotReport(te.getRobotID(), pose, currentPathIndex, state.getVelocity(), state.getPosition(), this.criticalPoint);
		}
	}

	public void delayIntegrationThread(int maxDelayInmillis) {
		this.maxDelayInMilis = maxDelayInmillis;
	}
	
	@Override
	public void run() {
		this.elapsedTrackingTime = 0.0;
		double deltaTime = 0.0;
		boolean atCP = false;
		int myRobotID = te.getRobotID();
		int myTEID = te.getID();
		
		while (true) {
						
			//End condition: passed the middle AND velocity < 0 AND no criticalPoint 			
			boolean skipIntegration = false;
			
			if (state.getVelocity() < 0.0) {
				if (criticalPoint == -1 && !atCP) {
					//set state to final position, just in case it didn't quite get there (it's certainly close enough)
					state = new State(totalDistance, 0.0);
					onPositionUpdate();
					break;
				}
								
				//Vel < 0 hence we are at CP, thus we need to skip integration
				if (!atCP /*&& getRobotReport().getPathIndex() == criticalPoint*/) {
					metaCSPLogger.info("At critical point (" + te.getComponent() + "): " + criticalPoint + " (" + getRobotReport().getPathIndex() + ")");
					atCP = true;
				}			
				skipIntegration = true;
			}

			//Compute deltaTime
			long timeStart = Calendar.getInstance().getTimeInMillis();
			
			//Update the robot's state via RK4 numerical integration
			if (!skipIntegration) {
				if (atCP) {
					metaCSPLogger.info("Resuming from critical point (" + te.getComponent() + ")");
					atCP = false;
				}
				integratePedestrianState(deltaTime);

			}
			
			//Do some user function on position update
			onPositionUpdate();
						
			//Sleep for tracking period
			int delay = trackingPeriodInMillis;
			if (maxDelayInMilis > 0) delay += rand.nextInt(maxDelayInMilis);
			try { Thread.sleep(delay); }
			catch (InterruptedException e) { e.printStackTrace(); }
			
			//Advance time to reflect how much we have slept (~ trackingPeriod)
			long deltaTimeInMillis = Calendar.getInstance().getTimeInMillis()-timeStart;
			deltaTime = deltaTimeInMillis/this.temporalResolution;
			elapsedTrackingTime += deltaTime;
		}
		
		//persevere with last path point in case listeners didn't catch it!
		long timerStart = getCurrentTimeInMillis();
		while (getCurrentTimeInMillis()-timerStart < WAIT_AMOUNT_AT_END) {
			//System.out.println("Waiting " + te.getComponent());
			try { Thread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		metaCSPLogger.info("RK4 tracking thread terminates (Robot " + myRobotID + ", TrajectoryEnvelope " + myTEID + ")");
	}

//	public static double[] computeDTs(Trajectory traj, double maxVel, double maxAccel) {
//		double distance = computeDistance(traj, 0, traj.getPose().length-1);
//		State state = new State(0.0, 0.0);
//		double time = 0.0;
//		double deltaTime = 0.0001;
//		
//		ArrayList<Double> dts = new ArrayList<Double>();
//		HashMap<Integer,Double> times = new HashMap<Integer, Double>();
//		dts.add(0.0);
//		times.put(0, 0.0);
//		
//		//First compute time to stop (can do FW here...)
//		while (state.getPosition() < distance/2.0 && state.getVelocity() < maxVel) {
//			integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);
//			time += deltaTime;
//		}
//		double positionToSlowDown = distance-state.getPosition();
//		//System.out.println("Position to slow down is: " + MetaCSPLogging.printDouble(positionToSlowDown,4));
//
//		state = new State(0.0, 0.0);
//		time = 0.0;
//		while (true) {
//			if (state.getPosition() >= distance/2.0 && state.getVelocity() < 0.0) break;
//			if (state.getPosition() >= positionToSlowDown) {
//				integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
//			}
//			else {
//				integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);				
//			}
//			//System.out.println("Time: " + time + " " + rr);
//			//System.out.println("Time: " + MetaCSPLogging.printDouble(time,4) + "\tpos: " + MetaCSPLogging.printDouble(state.getPosition(),4) + "\tvel: " + MetaCSPLogging.printDouble(state.getVelocity(),4));
//			time += deltaTime;
//			RobotReport rr = getRobotReport(traj, state);
//			if (!times.containsKey(rr.getPathIndex())) {
//				times.put(rr.getPathIndex(), time);
//				dts.add(time-times.get(rr.getPathIndex()-1));
//			}
//		}
//		if (dts.size() < traj.getPose().length) {
//			times.put(traj.getPose().length-1, time);		
//			dts.add(time-times.get(traj.getPose().length-2));
//		}
//		
//		//System.out.println("Time: " + MetaCSPLogging.printDouble(time,4) + "\tpos: " + MetaCSPLogging.printDouble(state.getPosition(),4) + "\tvel: " + MetaCSPLogging.printDouble(state.getVelocity(),4));
//		
//		double[] ret = new double[dts.size()];
//		for (int i = 0; i < dts.size(); i++) ret[i] = dts.get(i);
//		return ret;
//
//	}

	

}
