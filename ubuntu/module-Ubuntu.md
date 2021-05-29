# Week 1.1

[SLAM-Course-01-introduction to Robot Mapping](https://www.youtube.com/watch?v=wVsfCnyt5jA&list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_&index=3)

## Robot Mapping

* **Robot** - a device, that moves through the environment
* **Mapping** - modeling the environment

### Related terms

* State Estimation 
* Localization - estimating the robot's location
* Mapping - building a map
* SLAM - building a map and localizing the robot simultaneously 
* Navigation
* Motion Planning

## SLAM

* SLAM is the basis for most navigation
* SLAM is a chicken-or-egg problem
  * a map is needed for localizaion
  * a pose estimate is needed for mapping

### Definition of the SLAM Problem

- #### Given

  - The robot's controls (These are commands which we physically send to the robot like drive a metre forward).

    odometry is like a feedback to the command.
    $$
    u_{1:T} = {u_1,u_2,u_3,u_4....,u_T}
    $$
    
  - 
  
  * Observations : These can be laser scans, or it can be a camera image.
    $$
    z_{1:T} = {z_1,z_2,z_3,z_4....,z_T}
    $$

* Wanted 

  * Map of the environment

    $m$

  * Path of the robot
    $$
    x_{0:T} = {x_0,x_1,x_2,x_3,x_4....,x_T}
    $$

### Probabilistic Approach

* There are uncertainty in the robot's motions and observations

* So we use the probabilistic theory to explicitly represent the uncertainty
  $$
  p(x_{0:T},m | z_{1:T},u_{1:T})
  $$

### Full SLAM vs. Online SLAM

* Full SLAM estimates the entire path 
  $$
  p(x_{0:T},m | z_{1:T},u_{1:T})
  $$

* Online SLAM seeks to recover only the most recent pose

  * It means marginalising out the previous poses

  $$
  p(x_t,m | z_{1:T},u_{1:T}) \\ 
  p(x_t,m|z_{1:t},u_{1:t}) = \int_{x_0}{} ... \int_{x_{t-1}}{} p(x_{0:t},m | z_{1:T},u_{1:T}) dx_{t-1} .... dx_0
  $$

  *  Integrals are typically solved recursively, one at a time

### Why is SLAM a Hard Problem?

* The mapping between observations and the map is unknown. So  if a set of observation is made then the uncertainty in the position  of Robot can cause a totally different map to be perceived by the Robot. 
* Picking wrong data associations can have catastrophic consequences (divergence)

### Taxonomy of the SLAM Problem

1. Volumetric SLAM - It recognises the obstacles in a volumetric manner. All obstacles are marked grey and free spaces as white

2. feature-based SLAM - It recognises the obstacles as distinct landmarks. Hard to do because it exactly determines the identity of an object which may be difficult to achieve due to constraints of sensor.

1. Topological SLAM - Just the route not exactly similar geometry wise
2. Geometric SLAM - Exact geometry

1. Known correspondence
2. Unknown correspondence

1. Static Environment 
2. Dynamic Environment

1. Small uncertainty
2. Large uncertainty

1. Passive SLAM - There is a fixed data stream.
2. Active SLAM -  The robot makes its own decision of where to go to make a better map of the environment

### Three main Paradigms

* Kalman filter
* Particle filter
* Graph-based

### Motion and Observation model

#### Motion Model

It describes the relative motion of the robot
$$
p(x_t| x_{t-1},u_t)
$$

* Gaussian Model

* Non-Gaussian model

  Read 

  1. "Probabilistic Robotics", Chapter 5 
2.  "Introduction to mobile robotics", chapter 6

#### Observation Model

It relates the measurements with the robot's position
$$
p(z_t | x_t)
$$

* Gaussian model
* Non-Gaussian model

Read

1. "Probabilistic Robotics", Chapter 6
2.  "Introduction to mobile robotics", chapter 7

# Week 1.2

## Sensor Fusion ( Matlab )

[Understanding Sensor Fusion and Tracking](https://www.youtube.com/watch?v=6qV3YjFppuc)

Combining two or more data sources in a way that generates a better  understanding of the system

### 4 ways sensor fusion can help

1. it increase the quality of the data

   4 sensors = $\frac{1}{2}$ noise  (inverse sqrt relation)

   We can also use two different sensor types to reduce noise

2.  It can increase reliability

   We have backup if anyone fails

3. It can estimate unmeasured states
   A camera can't measure distance but a combination of two cameras can measure the distance

4. It can increase the coverage area


![Screenshot from 2021-03-06 18-46-57](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-03-06 18-46-57.png)

## AHRS

[Fusing an Accel, Mag, and Gyro to Estimation Orientation](https://www.youtube.com/watch?v=0rlvvYgmTvI&t=0s)

Sensor Fusion to estimate orientation attitude and heading reference system (AHRS)
We will use magnetometer + accelerometer + gyro

### Orientation

* reference frame
* specific rotation
  * row, pitch,yaw
  * direction cosine matrix
  * quaternion

#### Magnetometer

* Hard iron source would generate own magnetic field and would lead to a displaced sphere

* Soft iron source would distort magnetic field leading  to an ellipse instead of a sphere.

* Callliberation of magnetometer
  $$
  X_{corrected} = (x-b).A
  $$
  b : hard iron bias (3x1 vector)

  A : soft iron distortion (3x3 matrix)

   
#### Accelerometer

To estimate the orientation for a static object we can use accelerometer with correction but often we need to know the orientation of something which is rotating and accelerating. So we need to merge gyro.

![Screenshot from 2021-05-08 13-04-21](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-05-08 13-04-21.png)

![Screenshot from 2021-05-08 12-52-46](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-05-08 12-52-46.png)

#### Gyro (Dead Reckoning)

So since we have two was to estimate the orientation of an object (acce + mag and gyro ), both with their own pros and cons , we can use sensor fusion to combine these two estimates in a way that emphasises each of their strength and minimizes their weaknesses.

 ![Screenshot from 2021-05-08 12-56-06](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-05-08 12-56-06.png)

![Screenshot from 2021-05-08 13-03-00](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-05-08 13-03-00.png)

## Fusing a GPS and and IMU to estimate Pose

[Fusing a GPS and and IMU to estimate Pose](https://www.youtube.com/watch?v=hN8dL55rP5I&t=0s)

We combined the sensors in an IMU (inertial measurement unit) to predict an objects orientation.
Now we will combine GPS with IMU to get the postion and velocity as well

For determining position of a car GPS reading a few meters off is Ok But if we are following a fast trajectory through obstacles with drones then a few metres is not ok then we need to combine GOS with an IMU.

### Kalman Filter

We can measure a state directly or we can use our knowledge to predict the state in future. Combining both the results would give a better estimate of the result

## Tracking a single Object with an IMM filter

[Tracking a single Object with an IMM filter](https://www.youtube.com/watch?v=hJG08iWlres&t=0s)

We are switching from localization and positioning to single object tracking. To do so we will replace  a Single model estimation filter(often a kalman filter) with an interacting multiple model filter. 

![Screenshot from 2021-03-08 13-55-20](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-03-08 13-55-20.png)

Motion comes from

1. The dynamics and kinematics of the system
2. The commanded and known inputs
3. The unknown and random inputs

There are two types of tracking:

1. Cooperative Tracking - A cooperative object shares information with the tracking filter so tracking a cooperative object is similar to flying it ourselves 
2. Uncooperative Tracking - In this case control inputs are unknown.

![Screenshot from 2021-03-08 22-49-35](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-03-08 22-49-35.png)

![Screenshot from 2021-03-08 22-49-41](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-03-08 22-49-41.png)

### Multiple Model Filter

So in this case all three filter predictions are combined(constant velocity, constant turn, constant acc), one which gives closest output is trusted more next time

![Screenshot from 2021-03-08 23-12-49](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-03-08 23-12-49.png)

Why cannot we run on an IMM with a million model:

1. Costly 

     *  computational cost
     *  Models need to be set up and initialised correctly 
     
2. Performance hit
   
     * More models means more transitions
     * Harder to know when to make transition
     
     
     
     Usually 3-4 model suffice.(At max 10)

## Multiple Object Tracking

 [How to Track Multiple Objects at Once](https://www.youtube.com/watch?v=IIt1LHIHYc4&t=0s)

When uncertainty is high or objects are close to one another tracking every object independently doesn't work.

![Screenshot from 2021-03-08 23-41-45](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-03-08 23-41-45.png)

# Week 2.1

## Kalman Filter

[Kalman filter introdction](https://www.youtube.com/playlist?list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr)

[Important Article for Kalman filter](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

A kalman filter is an optimal estimation algorithm

### Temperature measurement in a rocket

To get the internal temperature of a chamber, we cannot directly use a sensor inside the chamber as it would melt instead we can measure a temperature outside the chamber and use the Kalman Filter to get the $T_{in}$ 

* IMU - measures acceleration and angular velocity of the car.
* Odometer - provides the relative position of the car.
* GPS receiver - provides the absolute position of the car.

### Navigation system of the car

IMU measures acceleration and angular velocity 
Odometer provides the relative position of the car
GPS receiver provides the absolute position of the car.

![Screenshot from 2021-03-13 19-41-05](Week 1.assets/Screenshot from 2021-03-13 19-41-05.png)

![Screenshot from 2021-03-13 19-40-30](Week 1.assets/Screenshot from 2021-03-13 19-40-30.png)

Kalman filter is used when;

* The variables of interest can only be measured indirectly
* Measurements are available from various sensors but might be subject to noise.

![Screenshot from 2021-03-13 19-46-50](Week 1.assets/Screenshot from 2021-03-13 19-46-50.png)

$u$ : input
$x$ : measurement to be calculated
$y$ : measurement known

$e_{obs} = x - \hat{x}$ 
$e_{obs}(t) = e^{(A-KC)t}e_{obs}(0)$

![Screenshot from 2021-03-13 19-50-11](Week 1.assets/Screenshot from 2021-03-13 19-50-11.png)

![Screenshot from 2021-03-13 19-59-16](Week 1.assets/Screenshot from 2021-03-13 19-59-16.png)

![Screenshot from 2021-03-13 20-02-10](Week 1.assets/Screenshot from 2021-03-13 20-02-10.png)

## Kalman Filter algorithm

[How Kalman Filter Works](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

State Observer : 
$\hat x_{k+1} = A\hat x_k + Bu_k + K(y_k - C\hat x_k)$		Deterministic system
$\hat x_k = A\hat x_{k-1} + Bu_k + K_k(y_k - C(A\hat x_{k-1} + B u_k))$ 		Stochastic system

$K_k$ is called the kalman gain and it determines how heavily the measurement and the apriori estimate contributes to the calculation of $\hat x_k$.

![Screenshot from 2021-03-13 20-43-09](Week 1.assets/Screenshot from 2021-03-13 20-43-09.png)

If the measurement noise is small it contributes more to the calculation of $\hat x_k$ while when the error in the apriori estimate is small, the apriori estimate is trusted more for the calculation of $\hat x_k$ 

## Kalman Filter for non linear system

* If the state function is linear then after undergoing the linear transformation, the distribution maintains its Gaussian property.
* If f(x) is non linear the resulting state distribution is not gaussian

 ### Extended Kalman Filter

EKF is not optimal if the system is highly non linear. In that case we incorporate Unscented Kalman Filters

![Screenshot from 2021-03-13 21-16-14](Week 1.assets/Screenshot from 2021-03-13 21-16-14.png)

### Unscented Kalman Filter

In this model the sigma points are propagated through the nonlinear system model

![Screenshot from 2021-03-13 21-15-54](Week 1.assets/Screenshot from 2021-03-13 21-15-54.png)

### Particle Filter

It approximates any arbitrary distribution that is not explicitly a Gaussian distribution. So the number of particles (sigma points) that a particle filter needs is much larger than you'd need for an  unscented Kalman filter.

![Screenshot from 2021-03-13 21-16-04](Week 1.assets/Screenshot from 2021-03-13 21-16-04.png)

![Screenshot from 2021-03-13 21-01-20](Week 1.assets/Screenshot from 2021-03-13 21-01-20.png)

# Week 2.2

## Bayes Filter

### State Estimation

Estimating the state $x$ of a system given observations $z$ and controls $u$
Goal : $p(x | z,u)$
$$
\begin{align*}
	bel(x_t) &= p(x_t|z_{1:t},u_{1:t})\\
    &= \eta p(z_t|x_t,z_{1:t-1},u_{1:t}) p(x_t|z_{1:t},u_{1:t})\\
    &= \eta p(z_t|x_t)p(x_t|z_{1:t-1},u_{1:t})\\
    &= \eta p(z_t|x_t) \int_{x_{t-1}}p(x_t|x_{t-1},z_{1:t-1},u_{1:t}) p(x_{t-1}|z_{1:t-1},u_{1:t})dx_{t-1}\\
    &= \eta p(z_t|x_t) \int_{x_{t-1}}p(x_t|x_{t-1},u_{1:t}) p(x_{t-1}|z_{1:t-1},u_{1:t})dx_{t-1}\\
    &= \eta p(z_t|x_t) \int_{x_{t-1}}p(x_t|x_{t-1},u_{1:t}) p(x_{t-1}|z_{1:t-1},u_{1:t-1})dx_{t-1}\\
    &= \eta p(z_t|x_t) \int_{x_{t-1}}p(x_t|x_{t-1},u_{1:t}) bel(x_{t-1})dx_{t-1}\\
\end{align*}
$$



### Perdiction and Correction Step

* Prediction Step

  $ \bar{be}l(x_t) = \eta p( z_t | x_t ) {bel}(x_t) $

* Correction Step

  $ bel(x_t) = \eta p( z_t | x_t ) \bar {bel} (x_t) $

In practice, one often finds two types of motion models:

* Odometry-based
* Velocity-based

![Screenshot from 2021-03-21 23-16-53](Week 1.assets/Screenshot from 2021-03-21 23-16-53.png)

# Week 2.3

## Least squares

### The Squared error criterion and the method of least squares

The most probable value of the unknown quantities will be that in which the sum of the squares of the differences between the actually observed and the computed values multiplied by the numbers that measure the degree of precision is a minimum

* A simple example of a resistor

  Let x be the resistance. Assume it a constant but unknown
  We make measurements, y, of the resistance.
  If we model our measurement as corrupted with noise v then y = x + v

  The best estimate of x would be given by the value of x which would minimize the squared error criterion (cost function)

$$
\hat x_{LS} = argmin_x(e_1^2 + e_2^2 +e_3^2 +e_4^2 )
$$

For a matrix $ H  \in R^{mxn}$  ,  $H^TH$  is invertible if and only if $m \geq n$

### Weighted Least Squares

In regular least squares, we implicitly assumed that each noise term was of equal variance. IID (Independent and Identically distributed)
But if we assume that each noise term has different covariance then we can define our noise covariance as follows.

![Screenshot from 2021-03-22 03-05-19](Week 1.assets/Screenshot from 2021-03-22 03-05-19.png)

![Screenshot from 2021-03-22 03-05-19](Week 1.assets/Screenshot from 2021-03-22 03-08-10.png)

### Recursive Least Squares

Till now we collected a batch of measurement and we used those measurements to compute our estimated quantities of interest.

What to do if we have a stream of data. Like if we have measurements 10 times per second.

* In this case we can use a recursive method one that keeps a running estimate of the optional parameter for all of the measurements that we've collected up to the previous time step and then update that estimate given the measurement at the current time step.

$$
\hat x_1 = (H^TR^{-1}H)^{-1}R^{-1}y_1 \\
\hat x_2 = (H^TR^{-1}H)^{-1}R^{-1}y_{1:2}
$$

 * Linear Recursive Estimator

$$
\hat x_k = \hat x_{k-1} + K_k(y_k - H_k\hat x_{k-1})
$$

Here K is Estimator gain matrix and the term in the bracket is called the innovation.
$$
K_k = P_{k-1} H_k^T(H_kP_{k-1}H_k^T + R_k)^{-1}\\
P_k = (1 - K_kH_k)P_{k-1}
$$
$P_k$ is the Estimator covariance matrix.

Recursive Least Squares minimizes the (co)variance of the parameter(s) at the current time.

![Screenshot from 2021-05-08 16-06-54](/home/utkarsh/Documents/iitb-racing/Week 1/Week 1.assets/Screenshot from 2021-05-08 16-06-54.png)

### Least Squares and the Method of Maximum Likelihood

Why we use squared errors?

Squared errors involves relatively standard algebra.

* The maximal likelihood estimate (MLE) is given by
  $$
  \hat x_{MLE} = argmax \hspace{3mm} p(y|x)
  $$
  because logarithm is a monotonically increasing function.
  we can instead minimise 
  $$
  \hat x_{MLE} = argmax \hspace{2 mm} log\hspace{1 mm}p(y|x)
  $$

Minimizing the sum of squared errors is equivalent to Maximizing the likelihood of a set of measurements assuming that the measurements are corrupted by additive, independent Gaussian noise that's of equal variance

If we maintain the same assumptions, but change the variance of the Gaussian noise for each measurement, we can arrive at the same criterion as we saw in weighted least square.

* Least squares estimates are significantly affected by outliers.
* Central Limit Theorem states that When independent random variables are added, their normalized sum tends towards a normal distribution.



# Week 3.1

* ROS Node

  * Official definiton: A node isn't much more than an exectable file within a ROS package. ROS nodes use a ROS client  library to communicate with other nodes. Nodes can publish or subscribe to a Topic. Nodes can also provide or use a Service.

* ROS:

  ROS Publisher: Publisher is a node which publishes (post) some data on Topics. It means it send some data to other Nodes.

  ROS Subscriber: Subscriber is a Node which subscribes to (receives data from) Topics. It means it receives some data from other nodes.