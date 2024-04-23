package edu.tufts.hrilab.diarcros.msg;


import org.ros.node.ConnectedNode;

import java.io.Serializable;

/**
 * ROS Time representation. Time and Time are primitive types in ROS. ROS
 * represents each as two 32-bit integers: seconds and nanoseconds since epoch.
 *
 * @author Jason Wolfe
 * @author kwc@willowgarage.com (Ken Conley)
 * @see "http://www.ros.org/wiki/msg"
 */
public class Time implements Comparable<Time>, Serializable {
    static final java.lang.String _TYPE = "std_msgs/Time";
    static final java.lang.String _DEFINITION = "time data\n";

    public int secs;
    public int nsecs;

    public Time() {
        secs = 0;
        nsecs = 0;
    }

    public Time(int secs, int nsecs) {
        this.secs = secs;
        this.nsecs = nsecs;
        normalize();
    }

    public Time(double secs) {
        this.secs = (int) secs;
        this.nsecs = (int) ((secs - this.secs) * 1000000000);
        normalize();
    }

    public Time(Time t) {
        this.secs = t.secs;
        this.nsecs = t.nsecs;
    }

    public Time add(Duration d) {
        return new Time(secs + d.secs, nsecs + d.nsecs);
    }

    public Time subtract(Duration d) {
        return new Time(secs - d.secs, nsecs - d.nsecs);
    }

    public Duration subtract(Time t) {
        return new Duration(secs - t.secs, nsecs - t.nsecs);
    }

    public static Time fromMillis(long timeInMillis) {
        int secs = (int) (timeInMillis / 1000);
        int nsecs = (int) (timeInMillis % 1000) * 1000000;
        return new Time(secs, nsecs);
    }

    public static Time fromNano(long timeInNs) {
        int secs = (int) (timeInNs / 1000000000);
        int nsecs = (int) (timeInNs % 1000000000);
        return new Time(secs, nsecs);
    }

    @Override
        public String toString() {
        return secs + ":" + nsecs;
    }

    public double toSeconds() {
        return totalNsecs() / 1e9;
    }

    public long totalNsecs() {
        return ((long) secs) * 1000000000 + nsecs;
    }

    public boolean isZero() {
        return totalNsecs() == 0;
    }

    public void normalize() {
        while (nsecs < 0) {
            nsecs += 1000000000;
            secs -= 1;
        }
        while (nsecs >= 1000000000) {
            nsecs -= 1000000000;
            secs += 1;
        }
    }

    @Override
        public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + nsecs;
        result = prime * result + secs;
        return result;
    }

    /**
     * Check for equality between {@link Time} objects.
     * <p/>
     * This method does not normalize {@link Time} representations, so fields must match
     * exactly.
     */
    @Override
        public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null) return false;
        if (getClass() != obj.getClass()) return false;
        Time other = (Time) obj;
        if (nsecs != other.nsecs) return false;
        if (secs != other.secs) return false;
        return true;
    }

    @Override
        public int compareTo(Time t) {
        if ((secs > t.secs) || ((secs == t.secs) && nsecs > t.nsecs)) {
            return 1;
        }
        if ((secs == t.secs) && (nsecs == t.nsecs)) {
            return 0;
        }
        return -1;
    }

    public static Time toAde(org.ros.message.Time val) {
        Time _time = new Time();
        _time.secs = val.secs;
        _time.nsecs = val.nsecs;
        return _time;
    }

    public static org.ros.message.Time toRos(Time val, org.ros.message.Time msg) {
        msg.secs = val.secs;
        msg.nsecs = val.nsecs;
        return msg;
    }

    public static org.ros.message.Time toRos(Time val, ConnectedNode node) {
        org.ros.message.Time time = node.getTopicMessageFactory().newFromType(_TYPE);
        time.secs = val.secs;
        time.nsecs = val.nsecs;
        return time;
    }
}
