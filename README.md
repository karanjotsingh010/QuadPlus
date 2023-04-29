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
