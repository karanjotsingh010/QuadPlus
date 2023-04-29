# QuadPlus project
# Karanjot Singh

Aerial manipulation robots offer the possibility of 3-dimensional workspace compared to their counterparts working on ground. This requires the aerial robot to be highly
maneuverable and capable of producing force in a desired direction for contact-based tasks. Underactuated quadrotors with coplanar propellers are useful for their compactness but offer
limited maneuverability. A bi-axial propeller tilting quadrotor augments the maneuverability using extensive thrust vectoring. However, it presents various challenges in the form of stable
attitude tracking, specially in case of external disturbances and actuator saturation. Therefore, this paper presents a cascaded control methodology designed to overcome these challenges.
Incremental control allocation is used for velocity control to handle external wind disturbances. A new approach for dealing with actuator saturation and redistributing the control effort to
redundant actuators is proposed. The controller is shown to be effective in handling hover and trajectory tracking in a desired hyperplane with accurate attitude tracking. Stable hover flight
is also achieved in the presence of external wind disturbance and actuator saturation. Furthermore, we also demonstrate the efficacy of the system in handling contact-based tasks. The
controller is able to demonstrate the capabilities of the bi-axial tilting platform, QuadPlus, used in this work.

## **Hardware Setup**
---------------------

![hardware_setup](https://user-images.githubusercontent.com/58835285/235308345-72aa97c6-165a-4d45-a116-00c172d2a6cd.png)

## **Applications**
---------------------
![Inspection_pic](https://user-images.githubusercontent.com/58835285/235308241-ad4ace15-f4b3-4aff-b50f-76429a826fa3.JPG)
![Pipeline_inspection](https://user-images.githubusercontent.com/58835285/235308249-d8762652-9487-4c75-b6bf-48edd1ad8ba0.png)

## **Trajectory tracking**
![Roll20_traj](https://user-images.githubusercontent.com/58835285/235308267-05ea2ea2-915f-4ef7-882a-4a7fdd11a1f3.png)

## **Wind resistance (with arbitrary attitude)**
![Roll20_wind_hover](https://user-images.githubusercontent.com/58835285/235308445-785cd548-087e-4ac7-b016-7b0f639a4192.png)
