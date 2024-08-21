package org.firstinspires.ftc.teamcode.Utils;

import java.util.Comparator;
import java.util.PriorityQueue;

public class UpdateQueue {
    PriorityQueue<Update> pq;

    class UpdateComparator implements Comparator<Update> {
        public int compare(Update up1,Update up2){
            if(up1.basePriority < up2.basePriority){
                return 1;
            }else if(up1.basePriority > up2.basePriority) {
                return -1;
            }
            return 0;
        }
    }

    public UpdateQueue() {
        pq = new PriorityQueue<Update>(new UpdateComparator());
    }

    public int getSize() {
        return pq.size();
    }

    public void addUpdate(Update updt) {
        pq.add(updt);
    }

    public void run() {
        while (!pq.isEmpty()) {
            pq.peek().action.run();
        }
    }
}
