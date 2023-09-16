# BDST-eVTOL-Aircraft
This repository contains the build, design, simulation, and test of an autonomous Electric Vertical Take-Off and Landing(eVTOL) vehicle from scratch.

#  Nature, Observation, and Ornithology
The urge to look up and wonder "how?" creatures fly is so innate to homo sapiens that one misses the entirety of the beauty of flight during questioning. Quantum Mechanics states that the observer crashes the wave function to a wave. Schrodinger's not-so-funny portrayal of a cat in the box implied curiosity killed the cat. (Meow! Ouch!)

So as an observer will it be safe to ask: Can one even observe flight just as it is? No image, no projections, literally nothing, just observe, observe, observe, and voila! Your entire consciousness is filled with what lies(interesting word choice) ahead of you. You lose the sense of time, space, relationship with your loved ones, hunger, and the most important thing in the world i.e. "YOU". Do you know that you may reach a state where "you are the world and the world is you!"? Can one even ponder this? Has anyone ever experienced this in oneself? 

If yes, then maybe that was the whole reason why humans started studying nature and someone created the words "Ornithology" and "Aviation". Some wise homosapien once said, "Words create Reality." 😮 So be it! Let's explore a reality relating to aviation and ornithology.

**Ornithology** derives from the 16th-century Latin word "ornithologia" translating to "bird science" from the Greek word "ornis"(bird) and "logos"(theory, science) **meaning the scientific study of birds.**

**Aviation** derives from the Latin word "avis" translating to "bird" and ** meaning relating to the activity of flying.**

#### Since the topic of interest is vertical take-off and landing aircraft, first let's look at some birds capable of taking off/hovering/landing vertically.

### Birds Capable of Taking-off/Hovering/Landing Vertically :

<img width="1482" alt="Duck" src="https://github.com/Praful22/BDSMTEvtol-Aircraft/blob/main/ducktakingoff.jpeg">

### Duck

<img width="1482" alt="HummingBird" src="https://github.com/Praful22/BDSMTEvtol-Aircraft/blob/main/Ruby-throated-hummingbird-vertical-takeoff.jpeg">

### Hummingbird

<img width="1482" alt="Kestrel" src="https://github.com/Praful22/BDSMTEvtol-Aircraft/blob/main/Common-Kestrel-2.jpg">

### Kestrel

<img width="1482" alt="Harrier" src="https://github.com/Praful22/BDSMTEvtol-Aircraft/blob/main/Harrier.jpg">

### Harrier

<img width="1482" alt="Osprey" src="https://github.com/Praful22/BDSMTEvtol-Aircraft/blob/main/OspreyNASA.jpg">

### Osprey

<img width="1482" alt="Kingfisher" src="https://github.com/Praful22/BDSMTEvtol-Aircraft/blob/main/px671271-image-kwvxl5r3.jpg">

### Kingfisher


# Sketch of a Commercial eVTOL Aircraft
<img width="1482" alt="Cora" src="https://github.com/Praful22/BDSMTEvtol-Aircraft/blob/main/IMG_5940.jpg">

# Aircraft Flight Dynamics Equations of Motions

1. An aircraft is treated as a rigid body whose dynamics are comprised of three translational and three rotational degrees of freedom, hence 6-DoF motion.

2. Body axes coordinate frame is fixed at the aircraft's center of gravity (CG).

Motion can be described by:
1. Translational Motion:

    a. Forward Velocity $\textit{u}$ (positive along the fuselage-body x-axis)

    b. Lateral Velocity $\textit{v}$ (positive along the right-wing-body y-axis)

    c. Vertical Velocity $\textit{w}$ (positive down and along the body z-axis)


2. Rotational degrees of freedom representing rotational motion:

    a. Body roll rate $\textit{p}$ (around the x-axis)

    b. Body pitch rate $\textit{q}$ (around the y-axis)

    c. Body yaw rate $\textit{r}$ (around the z-axis)

   Caveat: Positive angular rates(p,q,r) result in the counterclockwise rotations around their respective axis (x,y,z).

The 6-DoF aircraft equations of motion may be modeled as:

Translational Degree of Freedom: 
```math
\begin{aligned}
& m\left(\begin{array}{c}
\dot{u} \\
\dot{v} \\
\dot{w}
\end{array}\right)=-\left[\left(\begin{array}{c}
p \\
q \\
r
\end{array}\right) \times\left(\begin{array}{c}
u \\
v \\
w
\end{array}\right)\right]+\left(\begin{array}{c}
F_x \\
F_y \\
F_z
\end{array}\right)
+ m \underbrace{\underbrace{\|\vec{g}\|}_g\left(\begin{array}{c}
-\sin \theta \\
\cos \theta \sin \varphi \\
\cos \theta \cos \varphi
\end{array}\right)}_{\vec{g}} \\
\end{aligned}
```
Rotational Degree of Freedom:
```math
\begin{aligned}
 J\left(\begin{array}{c}
\dot{p} \\
\dot{q} \\
\dot{r}
\end{array}\right)=-\left[\left(\begin{array}{c}
p \\
q \\
r
\end{array}\right) \times J\left(\begin{array}{c}
p \\
q \\
r
\end{array}\right)\right]+\left(\begin{array}{c}
\bar{L} \\
M \\
N
\end{array}\right) \\
\end{aligned}
```
where, 

```math
\begin{align*}
m &\triangleq \text{aircraft mass} \\
J &\triangleq \text{Vehicle Inertia Matrix} \\
(F_x, F_y, F_z) &\triangleq \underbrace{\text{Body (x,y,z) components of forces}}_{\text{Due to Aerodynamics and Propulsion}} \\
(\bar{L}, M, N) &\triangleq \underbrace{\text{Body (x,y,z) components of Moments}}_{\text{Due to Aerodynamics and Propulsion}} \\
\vec{g} &\triangleq \text{gravity vector} \\
g &\triangleq \|\vec{g}\| = \text{Magnitude of the gravity vector}\\
\end{align*}
```
The gravity vector is expressed in the aircraft body axes coordinates in terms of the following:

```math
\begin{align*}
\phi &\triangleq \text{vehicle bank angle. Positive: Aircraft's right-wing down} \\
\theta &\triangleq \text{Pitch angle. Positive: Aircraft nose-up} \\
\psi &\triangleq \text{True heading angle. Positive: Clockwise rotation of the aircraft nose from the true north direction} \\
\end{align*}
```
The three Euler angles $(\phi, \theta, \psi)$ represent the inertial angular orientation of aircraft if treated as a 
rigid body moving in three-dimensional inertial space.

The following kinematics relation describes the dynamics of the Euler angles versus the aircraft body angular rates ($\textit{p},\textit{q},\textit{r}$):

```math
\begin{align*}
\left(\begin{array}{c}
\dot{\varphi} \\
\dot{\theta} \\
\dot{\psi}
\end{array}\right)=\left(\begin{array}{ccc}
1 & \sin \varphi \tan \theta & \cos \varphi \tan \theta \\
0 & \cos \varphi & -\sin \varphi \\
0 & \frac{\sin \varphi}{\cos \theta} & \frac{\cos \varphi}{\cos \theta}
\end{array}\right)\left(\begin{array}{l}
p \\
q \\
r
\end{array}\right)
\end{align*}
```

Checkout in more detail at: https://github.com/Praful22/AircraftFlightDynamics/tree/main

# Design Process
<img width="1482" alt="Design Process" src="https://github.com/Praful22/BDSMTEvtol-Aircraft/blob/main/Flying Vehicle.png">

# Control Map 
Credits to Control's Guru: Brain Douglas
![ControlChart drawio](https://github.com/Praful22/BDSMTEvtol-Aircraft/assets/65821250/7d4cab7b-6f64-484f-91a0-8a3adf30b62f)


The above-shown Map shows a general pathway to navigate throughout the project. 
One, many, or any methods/application strategies will be used in this project as deemed needed.
