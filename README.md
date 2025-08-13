1)      创建容器操作        sudo docker run -dt --name laps-container --privileged=true -v /home/xiale/laps_share:/fileInHost/ fe748aee9a42 /bin/bash

2)      进入容器操作        docker exec -it laps-container bash -c "cd /app/ns3-detnet-rdma-main/ns-3.33 && exec bash"
3)                          docker start laps-container
