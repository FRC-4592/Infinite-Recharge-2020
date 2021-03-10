package frc.robot.Lib.Loop;

import java.util.Vector;

// If you haven't noticed, the looper code is "inspired" by 254

public class MultiLooper implements Loopable {
	private Looper looper;
	private Vector<Loopable> loopables = new Vector<Loopable>();

	public MultiLooper(String name, double period, boolean use_notifier) {

		looper = new Looper(name, this, period);
	}

	public MultiLooper(String name, double period) {
		this(name, period, false);
	}

	public void update() {
		int i;
		for (i = 0; i < loopables.size(); ++i) {
			Loopable c = loopables.elementAt(i);
			if (c != null) {
				c.update();
			}
		}
	}

	public void start() {
		looper.start();
	}

	public void stop() {
		looper.stop();
	}

	public void addLoopable(Loopable c) {
		loopables.addElement(c);
	}
}