#!/bin/bash
for j in {1..3}
do
	./run.bat $j
done
python ./tools/sim-mesh-topo/sim-mesh-topo.py matrixLineTopo

