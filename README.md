# 3D EKF SLAM

<div align="center">
	<table>
	  <tr>
	    <td>Experiment</td>
	     <td>High Level reconstruction</td>
	     <td>Point cloud reconstruction</td>
	  </tr>
	  <tr>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/acelerado.gif" width=270></td>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/llmap.gif?raw=true" width=270></td>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/point.gif?raw=true" width=270></td>
	  </tr>
	 </table>
</div>

This repository contains the work from [Leonardo Mariga](https://github.com/leomariga) .

### [Thesis PDF](https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Publications/Leonardo_Mariga_MSc_Dissertation_ITA_2021.pdf) | [Videos](https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/tree/main/Videos) | [Presentation](https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Publications/Presentation/Leonardo%20Mariga%20Master%20presentation.pdf)

> **Solving the Simultaneous Localization and 3D Mapping Problem in Mobile Robotics Using Multi-Level Parameterized Representations**<br>
> Author: **Leonardo Mariga (ITA)**
> Advisor: Prof. Dr. Cairo Lúcio Nascimento Júnior (ITA)
>
> **Abstract:** *This work proposes an expansion of the traditional landmark-based EKF-SLAM for 3D indoor environments. It uses a mobile robot with an RGBD camera to decompose a scene and update its 3dof location. A point-cloud map is reconstructed by initially simplifying the scene into cylinders and planes. Then, its points are segmented and associated to their specific EKF feature. This enables to output a map using different levels of abstraction, which is helpful when working in applications with limited memory requirements. In our feature extraction method, the RANSAC is used for planar segmentation. The non-planar points are clustered using DB-SCAN and parameterized as cylinders after passing through validation gates. On each step, the classical EKF framework propagates and updates the features' states. This also triggers the point-cloud growth and increments the point-cloud of the observed feature. Our simulation results shows that this method successfully reconstructs maps, reducing the IAE criteria to 15% of trajectory error compared to the odometry. Finally, we sucessfully validate the real environment, using a Intel Realsense camera.*

## 3D EKF SLAM: how does it work?

An overview the algorithm is illustrated in figure below as a V-shaped process. The vertical axis represents map density (which is related to its level of abstraction) and the horizontal axis is the processing order. Starting as a point cloud input and going down to a simplified primitive and parameterized local scene, the features are associated and merged to a global map.

By doing that, the robot's pose is corrected and a global high level map can be generated using the parameterized feature. If the application chooses to carry the point cloud, it can create a point cloud or voxel-based maps as output.

<div align="center">
  <img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/overview.png" width=700><br>
</div>
 

## Results:

Simulation results shows that in spite of the odometry error slowly increasing and reaching 7 m, the estimated trajectory error is always within bounds of 1 m. As a quantification tool, we can use the  Integral Absolute Error (IAE) criteria to measure the overall error during the whole SLAM process. Table shows the IAE criteria for this case.The odometry path error is the error related to the concatenation of commands sent to the robot and the estimated path error is obtained from comparing the proposed algorithm with the real path. 

|                      | IAE Criteria |
|----------------------|--------------|
| Odometry path error  | 304.28       |
| Our estimation error | 49.82        |

<div align="center">
  <img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/errorstep.png" width=600><br>
</div>


<div align="center">
	<table>
	  <tr>
	    <td>Experiment</td>
	     <td>High Level reconstruction</td>
	     <td>Point cloud reconstruction</td>
	  </tr>
	  <tr>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/experiment.jpg" width=270></td>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/f3.png" width=270></td>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/f2.png" width=270></td>
	  </tr>
	  <tr>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/tb1.png" width=270></td>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/lixeira2.png" width=270></td>
	    <td><img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/lixeira1.png" width=270></td>
	  </tr>
	 </table>
</div>

Point cloud reconstruction semi-colered by feature:

<div align="center">
  <img src="https://github.com/Intelligent-Machines-Lab/3D_EKF_SLAM/blob/main/Videos/corredorpt2.png" width=800><br>
</div>

## Contact

Author: [Leonardo Mariga](https://github.com/leomariga) 
E-mail: leomariga@gmail.com

Feel free to contact me.