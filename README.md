1)      创建容器操作         sudo docker run -dt --name laps-container --privileged=true -v /home/xiale/laps_share:/file-in-ctr/ fe748aee9a42 /bin/bash

2)           docker exec -it laps-container bash -c "cd /app/ns3-detnet-rdma-main/ns-3.33 && exec bash"
3)                          docker start laps-container
4)                        python3 /file-in-ctr/executableFiles/C00003/run.py
5) set args --fileIdx=C00001_dragonfly_GoogleRPC2008_All-lr-0.1-lb-ecmp --outputFileDir=/file-in-ctr/outputFiles/C00001/ --inputFileDir=/file-in-ctr/inputFiles/C00001/ --topoFileName=/file-in-ctr/inputFiles/C00001/dragonfly/TOPO.txt --configFileName=/file-in-ctr/inputFiles/C00001/CONFIG_DCQCN.txt --simStartTimeInSec=0 --qlenMonitorIntervalInNs=10000000 --simEndTimeInSec=0.2 --flowLunchStartTimeInSec=0.01 --flowLunchEndTimeInSec=0.01 --lbsName=ecmp --flowletTimoutInUs=10 --loadRatioShift=1 --loadRatio=0.1 --ccMode=Dcqcn_mlx --screenDisplayInNs=10000000 --enablePfcMonitor=false --enableFctMonitor=true --enableQlenMonitor=false --enableQbbTrace=false --rdmaAppStartPort=1000 --testPktNum=1 --workloadFile=/file-in-ctr/inputFiles/workload/GoogleRPC2008.txt --patternFile=/file-in-ctr/inputFiles/C00001/dragonfly/TFC-All.txt --SMTFile=/file-in-ctr/inputFiles/C00001/dragonfly/SMT.txt --PITFile=/file-in-ctr/inputFiles/C00001/dragonfly/PIT.txt --PSTFile=/file-in-ctr/inputFiles/C00001/dragonfly/PST.txt --enableFlowCongestTest=true --enableLLMWorkLoadTest=false
6) set args --fileIdx=C00001_dragonfly_GoogleRPC2008_All-lr-0.1-lb-e2elaps \
--outputFileDir=/file-in-ctr/outputFiles/C00001/ \
--inputFileDir=/file-in-ctr/inputFiles/C00001/ \
--topoFileName=/file-in-ctr/inputFiles/C00001/dragonfly/TOPO.txt \
--configFileName=/file-in-ctr/inputFiles/C00001/CONFIG_DCQCN.txt \
--simStartTimeInSec=0 \
--qlenMonitorIntervalInNs=10000000 \
--simEndTimeInSec=0.2 \
--flowLunchStartTimeInSec=0.01 \
--flowLunchEndTimeInSec=0.01 \
--lbsName=e2elaps \
--flowletTimoutInUs=10 \
--loadRatioShift=1 \
--loadRatio=0.1 \
--ccMode=Laps \
--screenDisplayInNs=10000000 \
--enablePfcMonitor=false \
--enableFctMonitor=true \
--enableQlenMonitor=false \
--enableQbbTrace=false \
--rdmaAppStartPort=1000 \
--testPktNum=1 \
--workloadFile=/file-in-ctr/inputFiles/workload/GoogleRPC2008.txt \
--patternFile=/file-in-ctr/inputFiles/C00001/dragonfly/TFC-All.txt \
--SMTFile=/file-in-ctr/inputFiles/C00001/dragonfly/SMT.txt \
--PITFile=/file-in-ctr/inputFiles/C00001/dragonfly/laps-PIT.txt \
--PSTFile=/file-in-ctr/inputFiles/C00001/dragonfly/laps-PST.txt \
--enableFlowCongestTest=true \
--enableLLMWorkLoadTest=false \
--enablee2elapsPFC=true \
--enableRecordLBOutInfo=true
7)                docker cp /home/xiale/ns3/userdefinedfunction.cc laps-container:/app/ns3-detnet-rdma-main/ns-3.33/src/userdefinedfunction/model/
