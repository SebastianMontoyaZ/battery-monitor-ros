
## Save the Docker Image to .tar file

docker save battery-monitor -o $PWD/battery-monitor.tar

  

## Upload docker image to K3s cluster

sudo k3s ctr images import $PWD/battery-monitor.tar

  

# k3s-config
kubectl apply -f k3s-config/battery-monitor.yaml

the yaml file builds:

 - Namespace: ros-battery-monitor
 - Pod with the healthCheck system
 - Service for the pod
 - ServiceMonitor in "prometheus" namespace

