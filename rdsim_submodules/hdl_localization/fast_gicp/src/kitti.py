#!/usr/bin/python3
import os
import sys
import time
import numpy
import pygicp
from matplotlib import pyplot

def main():
	if len(sys.argv) < 2:
		print('usage: kitti.py /path/to/kitti/sequences/00/velodyne')
		return

                   
	seq_path = sys.argv[1]
	filenames = sorted([seq_path + '/' + x for x in os.listdir(seq_path) if x.endswith('.bin')])

                                                                       
	reg = pygicp.FastGICP()

                                                            
                         
                                           

	stamps = [time.time()]                       
	poses = [numpy.identity(4)]                    

	for i, filename in enumerate(filenames):
                                   
		points = numpy.fromfile(filename, dtype=numpy.float32).reshape(-1, 4)[:, :3]
		points = pygicp.downsample(points, 0.25)

		if i == 0:
			reg.set_input_target(points)
			delta = numpy.identity(4)
		else:
			reg.set_input_source(points)
			delta = reg.align()
			reg.swap_source_and_target()

                                                              
		poses.append(poses[-1].dot(delta))

                                           
		stamps = stamps[-9:] + [time.time()]
		print('fps:%.3f' % (len(stamps) / (stamps[-1] - stamps[0])))

                                 
		traj = numpy.array([x[:3, 3] for x in poses])

		if i % 30 == 0:
			pyplot.clf()
			pyplot.plot(traj[:, 0], traj[:, 1])
			pyplot.axis('equal')
			pyplot.pause(0.01)


if __name__ == '__main__':
	main()
