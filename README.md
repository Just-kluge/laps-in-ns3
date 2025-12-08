1. sudo docker exec -it laps-container bash -c "cd /app/ns3-detnet-rdma-main/ns-3.33 && exec bash"
   
7. python3 /file-in-ctr/executableFiles/C00001/run.py

6. sudo docker start laps-container


   
3. sudo docker exec -it laps-container-try-myself-idea bash -c "cd /app/ns3-detnet-rdma-main/ns-3.33 && exec bash"
   
5. python3 /file-in-ctr/executableFilesForMyIdea/C00001/run-ForMyIdea.py




8. ./waf build


9. sudo docker cp /home/super/xiaojinyu/copy-p2p/model/. laps-container:/app/ns3-detnet-rdma-main/ns-3.33/src/point-to-point/model/

10. ./waf --run "scratch/main --fileIdx=C00001_railOnly_DCTCP_CDF_All-lr-0.1-lb-ecmp --outputFileDir=/file-in-ctr/outputFiles/C00001/ --inputFileDir=/file-in-ctr/inputFiles/C00001/ --topoFileName=/file-in-ctr/inputFiles/C00001/railOnly/TOPO.txt --configFileName=/file-in-ctr/inputFiles/C00001/CONFIG_DCQCN.txt --simStartTimeInSec=0 --qlenMonitorIntervalInNs=10000000 --simEndTimeInSec=15 --flowLunchStartTimeInSec=0.01 --flowLunchEndTimeInSec=0.01 --lbsName=ecmp --flowletTimoutInUs=10 --loadRatioShift=1 --loadRatio=0.1 --ccMode=Dcqcn_mlx --screenDisplayInNs=10000000 --enablePfcMonitor=false --enableFctMonitor=true --enableQlenMonitor=false --enableQbbTrace=false --rdmaAppStartPort=1000 --testPktNum=1 --workloadFile=/file-in-ctr/inputFiles/workload/DCTCP_CDF.txt --patternFile=/file-in-ctr/inputFiles/C00001/railOnly/TFC-All.txt --SMTFile=/file-in-ctr/inputFiles/C00001/railOnly/SMT.txt --PITFile=/file-in-ctr/inputFiles/C00001/railOnly/PIT.txt --PSTFile=/file-in-ctr/inputFiles/C00001/railOnly/PST.txt --enableFlowCongestTest=True --enableLLMWorkLoadTest=false" --gdb




