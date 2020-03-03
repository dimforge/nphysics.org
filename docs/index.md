<center>
![nphysics](./img/logo_nphysics_full.svg)
</center>
<br/>
<center>
[![Join us on Discord](https://img.shields.io/discord/507548572338880513.svg?logo=discord&colorB=7289DA)](https://discord.gg/vt9DJSW)
[![CircleCI](https://circleci.com/gh/rustsim/nphysics.svg?style=svg)](https://circleci.com/gh/circleci/circleci-docs)
[![Crates.io Status](https://meritbadge.herokuapp.com/nphysics3d)](https://crates.io/crates/nphysics3d)
[![License (3-Clause BSD)](https://img.shields.io/badge/license-BSD%203--Clause-blue.svg?style=flat)](https://opensource.org/licenses/BSD-3-Clause)
<div style="text-align:center">
<br/>
<a href="https://www.patreon.com/bePatron?u=7111380" ><img src="/img/become_a_patron_button.png" alt="Become a Patron!" /></a>

-----

<span class="h1 headline">2D and 3D real-time physics engine</span>
<div></div>
<span class="subheadline">… for the [Rust](https://www.rust-lang.org) programming language.</span>
</center>

<br>

<table markdown="1">
<tr>
    <td>[![](/img/feature_rigid_body_simulation.svg)](../rigid_body_simulations_with_contacts)</td>
    <td style="vertical-align:middle">
    <a href="../rigid_body_simulations_with_contacts" id="no_highlight">
    <div>
    <big>**Rigid-body simulations with contacts**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Setup a physics world to simulate rigid solids with external forces and contacts. Includes dynamic, static,
    and kinematic rigid bodies, as well as colliders for collision detection.
    </div>
    </a>
    </td>
</tr>
<tr>
    <td>[![](/img/feature_multibody_and_constraints.svg)](../joint_constraints_and_multibodies)</td>
    <td style="vertical-align:middle">
    <a href="../joint_constraints_and_multibodies" id="no_highlight">
    <div>
    <big>**Joint constraints and multibodies**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Constrain the relative motion of two bodies either using forces (with joint constraints) or by
    reducing their relative degrees of freedom (with multibodies).
    </div>
    </a>
    </td>
</tr>
<tr>
    <td>[![](/img/feature_deformable_bodies.svg)](../deformable_bodies)</td>
    <td style="vertical-align:middle">
    <a href="../deformable_bodies" id="no_highlight">
    <div>
    <big>**Deformable bodies simulation**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Simulate deformable bodies that can interact with all other kinds of bodies implemented in **nphysics**.
    The computation of deformation is either based on a mass-spring system or a finite-element method.
    </div>
    </a>
    </td>
</tr>
<tr>
    <td>[![](/img/feature_contact_models.svg)](../contact_models)</td>
    <td style="vertical-align:middle">
    <a href="../contact_models" id="no_highlight">
    <div>
    <big>**Contact models**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Choose the appropriate contact model to achieve the desired level of accuracy and performance.
    Included are the Signorini and an approximate Signorini-Coulomb contact models.
    </div>
    </a>
    </td>
</tr>
<tr>
    <td>[![](/img/feature_event_handling_and_sensors.svg)](../interaction_handling_and_sensors)</td>
    <td style="vertical-align:middle">
    <a href="../interaction_handling_and_sensors" id="no_highlight">
    <div>
    <big>**Interaction handling and sensors**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Retrieve collision events to apply logic depending on the interaction between objects.
    Use a special type of collider, sensors, which won't interact physically with other objects
    but will generate proximity events.
    </div>
    </a>
    </td>
</tr>
<tr>
    <td>[![](/img/feature_continuous_collision_detection.svg)](../continuous_collision_detection)</td>
    <td style="vertical-align:middle">
    <a href="../continuous_collision_detection" id="no_highlight">
    <div>
    <big>**Continuous collision detection**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Don't miss any contact by enabling continuous collision detection (CCD) on any collider.
    Prevent the tunneling problem between fast-moving bodies, and don't miss any proximity
    event thanks to full CCD support on sensors.
    </div>
    </a>
    </td>
</tr>
<tr>
    <td>[![](/img/feature_performances_tuning.svg)](../performance_tuning)</td>
    <td style="vertical-align:middle">
    <a href="../performance_tuning" id="no_highlight">
    <div>
    <big>**Accuracy and performance tuning**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Choose the right compromise between accuracy and performance for your application.
    </div>
    </a>
    </td>
</tr>
<tr>
    <td>[![](/img/nphysics_testbed.svg)](../nphysics_testbed)</td>
    <td style="vertical-align:middle">
    <a href="../nphysics_testbed" id="no_highlight">
    <div>
    <big>**The nphysics testbed**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Setup a physical scene and use the **nphysics_testbed2d** or **nphysics_testbed3d** crate to display and
    interact with it.
    </div>
    </a>
    </td>
</tr>
<tr>
    <td>[![](/img/feature_wasm.svg)](../wasm_compatibility)</td>
    <td style="vertical-align:middle">
    <a href="../wasm_compatibility" id="no_highlight">
    <div>
    <big>**WASM compatibility**</big>
    <span class="home_dummy_link">&nbsp;<i class="fa fa-external-link" aria-hidden="true"></i></span>
    <br>
    Use **nphysics** on a project expected to run on a web browser.
    </div>
    </a>
    </td>
</tr>
</table>