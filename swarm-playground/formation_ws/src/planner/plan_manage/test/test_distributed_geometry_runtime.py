#!/usr/bin/env python3
"""Validate full-height snapshots and their independent freshness gate.

Fault injection stops planner 0 only on the dedicated localhost:11335 master.
Do not run this test against an existing experiment.
"""
import argparse
import json
import os
import time

import numpy as np
import rosnode
import rospy
from sensor_msgs.msg import PointCloud2
from ego_planner.msg import FormationAgentState, FormationEdgeEvidence


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-fault-injection',action='store_true')
    parser.add_argument('--timeout',type=float,default=75)
    args=parser.parse_args()
    if args.allow_fault_injection:
        assert os.environ.get('ROS_MASTER_URI')=='http://127.0.0.1:11335', 'Use the isolated test master'
    rospy.init_node('distributed_geometry_runtime',anonymous=True)
    snapshots,states,free,evidence={},{},{},{}
    high=set()
    subscribers=[]
    def snapshot(msg,drone):
        assert not msg.header.stamp.is_zero()
        snapshots[drone]=(time.monotonic(),msg.header.stamp.to_sec())
        if drone not in high and msg.width:
            z=next(f for f in msg.fields if f.name=='z')
            values=np.ndarray((msg.height,msg.width),dtype='<f4',buffer=msg.data,
                              offset=z.offset,strides=(msg.row_step,msg.point_step))
            if np.max(values)>1.9:high.add(drone)
    def state(msg):states[msg.drone_id]=(time.monotonic(),msg)
    def edge(msg):
        if msg.observer_id==0:evidence[msg.edge_index]=(time.monotonic(),msg)
    for drone in range(8):
        subscribers.append(rospy.Subscriber('/drone_%d_ego_planner_node/grid_map/formation_occupancy'%drone,
                            PointCloud2,snapshot,callback_args=drone,queue_size=1))
        subscribers.append(rospy.Subscriber('/drone_%d/observed_free'%drone,PointCloud2,
                            lambda msg,i=drone:free.update({i:time.monotonic()}),queue_size=1))
    subscribers.append(rospy.Subscriber('/swarm/agent_state',FormationAgentState,state,queue_size=64))
    subscribers.append(rospy.Subscriber('/swarm/edge_evidence',FormationEdgeEvidence,edge,queue_size=64))
    deadline=time.monotonic()+args.timeout
    while time.monotonic()<deadline:
        now=time.monotonic()
        all_past_wall=len(states)==8 and all(v[1].position[0]>0.5 for v in states.values())
        fresh=len(snapshots)==8 and all(now-v[0]<1.0 for v in snapshots.values())
        active=any(v[1].decision==FormationEdgeEvidence.KEEP for v in evidence.values())
        if all_past_wall and fresh and high==set(range(8)) and active:break
        time.sleep(0.1)
    else:
        raise AssertionError('snapshot/first-wall readiness failed: snapshots=%s high=%s positions=%s'%(
            sorted(snapshots),sorted(high),{i:list(v[1].position) for i,v in states.items()}))
    print(json.dumps({'full_height_snapshots':'PASS','first_wall_passed':sorted(states),
                      'positions':{i:list(v[1].position) for i,v in states.items()}}),flush=True)
    if not args.allow_fault_injection:return
    stopped=time.monotonic()
    _,failed=rosnode.kill_nodes(['/drone_0_ego_planner_node'])
    assert not failed,failed
    while time.monotonic()-stopped<8:
        now=time.monotonic()
        fresh_observation=now-states[0][0]<0.5 and now-free[0]<0.7
        snapshot_expired=now-snapshots[0][0]>1.0
        cut=len(evidence)==2 and all(v[0]>stopped+0.9 and
                                    v[1].decision==FormationEdgeEvidence.CUT for v in evidence.values())
        if snapshot_expired and fresh_observation and cut:
            print(json.dumps({'planner_snapshot_loss':'PASS','fresh_odom_and_observation':True,
                              'incident_edges_cut':sorted(evidence),
                              'seconds_after_planner_stop':now-stopped}),flush=True)
            return
        time.sleep(0.05)
    raise AssertionError('stale planner snapshot was not independently rejected')


if __name__=='__main__':main()
